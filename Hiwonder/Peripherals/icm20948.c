// ============================================================================
// File: Peripherals/icm20948.c
// Minimal ICM‑20948 (+ AK09916) driver implementation (polling)
// ============================================================================
#include "icm20948.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---- Conversions ------------------------------------------------------------
#define G_SI                       9.80665f
// Accel LSB/g: 2g=16384, 4g=8192, 8g=4096, 16g=2048
#define ACC_LSB_PER_G_4G           8192.0f
// Gyro LSB/(dps): 250=131, 500=65.5, 1000=32.8, 2000=16.4
#define GYR_LSB_PER_DPS_2000       16.4f
// AK09916 sensitivity: 0.15 µT/LSB
#define MAG_UT_PER_LSB             0.15f
// ICM‑20948 temperature: degC = temp_raw/333.87 + 21
#define TEMP_SENS                  333.87f
#define TEMP_OFFSET_C              21.0f

// Active 7‑bit I2C address (auto‑probed in begin)
static uint8_t s_addr = ICM20948_ADDR_69;

// ---- Low-level I2C helpers --------------------------------------------------
static inline HAL_StatusTypeDef i2c_wr(uint8_t reg, uint8_t val){
    return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(s_addr<<1), reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 5);
}
static inline HAL_StatusTypeDef i2c_rd(uint8_t reg, uint8_t* val){
    return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(s_addr<<1), reg, I2C_MEMADD_SIZE_8BIT, val, 1, 5);
}
static inline HAL_StatusTypeDef i2c_rdb(uint8_t reg, uint8_t* buf, uint16_t len){
    return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(s_addr<<1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, 5);
}
static inline int16_t be16(uint8_t msb, uint8_t lsb){ return (int16_t)((msb<<8) | lsb); }
static inline void ms(uint32_t x){ HAL_Delay(x); }
static inline int try_read_addr(uint8_t addr7, uint8_t reg, uint8_t* out){
    return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr7<<1), reg, I2C_MEMADD_SIZE_8BIT, out, 1, 5) == HAL_OK;
}
static inline int bank(uint8_t b){ return i2c_wr(ICM20948_REG_BANK_SEL, ICM20948_BANK(b)) == HAL_OK; }

// ---- AK09916 one‑shot write via SLV0 ---------------------------------------
static int ak09916_write(uint8_t reg, uint8_t val){
    if(!bank(ICM20948_BANK3)) return 0;
    if(i2c_wr(ICM20948_I2C_SLV0_ADDR, (AK09916_I2C_ADDR & 0x7F))!=HAL_OK) return 0; // write
    if(i2c_wr(ICM20948_I2C_SLV0_REG, reg)!=HAL_OK) return 0;
    if(i2c_wr(ICM20948_I2C_SLV0_DO, val)!=HAL_OK) return 0;
    if(i2c_wr(ICM20948_I2C_SLV0_CTRL, (uint8_t)(0x80 | 1))!=HAL_OK) return 0; // enable, len=1
    ms(2);
    (void)i2c_wr(ICM20948_I2C_SLV0_CTRL, 0x00); // disable again
    return 1;
}

// Configure continuous auto‑read of 9 bytes from AK09916: ST1..ST2 into EXT_SLV_SENS_00..08
static int ak09916_enable_auto_read(void){
    if(!bank(ICM20948_BANK3)) return 0;
    // RNW=1 (read)
    if(i2c_wr(ICM20948_I2C_SLV0_ADDR, (uint8_t)(0x80 | (AK09916_I2C_ADDR & 0x7F)))!=HAL_OK) return 0;
    if(i2c_wr(ICM20948_I2C_SLV0_REG, AK09916_REG_ST1)!=HAL_OK) return 0;
    if(i2c_wr(ICM20948_I2C_SLV0_CTRL, (uint8_t)(0x80 | 9))!=HAL_OK) return 0; // enable, len=9
    return bank(ICM20948_BANK0);
}

// ---- Public API -------------------------------------------------------------
bool icm20948_begin(void)
{
    uint8_t who = 0;

    // Probe WHO_AM_I at 0x69 then 0x68 (SparkFun often uses 0x69)
    s_addr = ICM20948_ADDR_69;
    if(!try_read_addr(s_addr, ICM20948_WHO_AM_I, &who) || who != 0xEA){
        s_addr = ICM20948_ADDR_68;
        if(!try_read_addr(s_addr, ICM20948_WHO_AM_I, &who) || who != 0xEA){
            return false;
        }
    }

    // Reset, wake
    if(!bank(ICM20948_BANK0)) return false;
    (void)i2c_wr(ICM20948_PWR_MGMT_1, 0x80);  // DEVICE_RESET
    ms(10);
    (void)i2c_wr(ICM20948_PWR_MGMT_1, 0x01);  // auto clk, exit sleep
    (void)i2c_wr(ICM20948_PWR_MGMT_2, 0x00);  // enable all axes

    // Configure ODR + ranges + DLPF
    if(!bank(ICM20948_BANK2)) return false;

    // Align ODR updates
    (void)i2c_wr(ICM20948_ODR_ALIGN_EN, 0x01);

    // Gyro: DLPF cfg=4 (~36 Hz NBW), FS_SEL=3 (±2000 dps), FCHOICE=1 (enable DLPF)
    (void)i2c_wr(ICM20948_GYRO_CONFIG_1, (uint8_t)((4u<<3) | (3u<<1) | 1u));
    // ~102 Hz ODR => div=10 (1125/(1+10) ≈ 102.27 Hz)
    (void)i2c_wr(ICM20948_GYRO_SMPLRT_DIV, 10);

    // Accel: DLPF cfg=4, FS_SEL=1 (±4 g), FCHOICE=1 (enable DLPF)
    (void)i2c_wr(ICM20948_ACCEL_CONFIG, (uint8_t)((4u<<3) | (1u<<1) | 1u));
    // ~102 Hz ODR (12‑bit divider = 10)
    (void)i2c_wr(ICM20948_ACCEL_SMPLRT_DIV_1, 0);
    (void)i2c_wr(ICM20948_ACCEL_SMPLRT_DIV_2, 10);

    // Temperature DLPF (optional): pick mid value
    (void)i2c_wr(ICM20948_TEMP_CONFIG, 3);

    // Enable internal I2C master for the magnetometer
    if(!bank(ICM20948_BANK0)) return false;
    (void)i2c_wr(ICM20948_USER_CTRL, 0x20);   // I2C_MST_EN

    if(!bank(ICM20948_BANK3)) return false;
    (void)i2c_wr(ICM20948_I2C_MST_ODR_CONFIG, 0x04); // pacing
    (void)i2c_wr(ICM20948_I2C_MST_CTRL, 0x07);       // master clk ~345 kHz

    // AK09916 soft reset, then set 100 Hz continuous mode
    (void)ak09916_write(AK09916_REG_CNTL3, AK09916_SOFT_RESET);
    ms(10);
    (void)ak09916_write(AK09916_REG_CNTL2, AK09916_MODE_CONT_100HZ);
    ms(1);

    if(!ak09916_enable_auto_read()) return false;

    return bank(ICM20948_BANK0);
}

bool icm20948_read_all(imu20948_raw_t* o)
{
    if(!o) return false;

    // Read Accel(6) + Gyro(6) + Temp(2) in one burst: 0x2D..0x3A (14 bytes)
    uint8_t buf[14];
    if(i2c_rdb(ICM20948_ACCEL_XOUT_H, buf, sizeof(buf)) != HAL_OK) return false;

    int16_t ax_raw = be16(buf[0], buf[1]);
    int16_t ay_raw = be16(buf[2], buf[3]);
    int16_t az_raw = be16(buf[4], buf[5]);

    int16_t gx_raw = be16(buf[6], buf[7]);   // GYRO_XOUT_H/L
    int16_t gy_raw = be16(buf[8], buf[9]);
    int16_t gz_raw = be16(buf[10], buf[11]);

    int16_t temp_raw = be16(buf[12], buf[13]);

    // External sensor block for mag: ST1 + HXL..HZH + TMPS + ST2 = 9 bytes
    uint8_t m[9];
    if(i2c_rdb(ICM20948_EXT_SLV_SENS_00, m, sizeof(m)) != HAL_OK) return false;

    // AK09916 data are little-endian per axis
    int16_t mx_raw = (int16_t)((m[2] << 8) | m[1]);
    int16_t my_raw = (int16_t)((m[4] << 8) | m[3]);
    int16_t mz_raw = (int16_t)((m[6] << 8) | m[5]);

    // Convert to SI (no timestamp)
    o->ax = (ax_raw / ACC_LSB_PER_G_4G) * G_SI;
    o->ay = (ay_raw / ACC_LSB_PER_G_4G) * G_SI;
    o->az = (az_raw / ACC_LSB_PER_G_4G) * G_SI;

    const float dps2rads = (float)M_PI / 180.0f;
    o->gx = (gx_raw / GYR_LSB_PER_DPS_2000) * dps2rads;
    o->gy = (gy_raw / GYR_LSB_PER_DPS_2000) * dps2rads;
    o->gz = (gz_raw / GYR_LSB_PER_DPS_2000) * dps2rads;

    // Magnetometer: microtesla
    o->mx = (mx_raw * MAG_UT_PER_LSB);
    o->my = (my_raw * MAG_UT_PER_LSB);
    o->mz = (mz_raw * MAG_UT_PER_LSB);

    // Temperature: °C
    o->temp_c = (temp_raw / TEMP_SENS) + TEMP_OFFSET_C;

    return true;
}
