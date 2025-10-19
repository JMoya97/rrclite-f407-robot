#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

// ---- I2C addresses (7-bit)
#define ICM20948_ADDR_68          0x68u
#define ICM20948_ADDR_69          0x69u

// ---- Bank select
#define ICM20948_REG_BANK_SEL     0x7F
#define ICM20948_BANK(b)          ((uint8_t)((b) << 4))
#define ICM20948_BANK0            0u
#define ICM20948_BANK1            1u
#define ICM20948_BANK2            2u
#define ICM20948_BANK3            3u

/* ---- Bank 0 ---- */
#define ICM20948_WHO_AM_I         0x00  // expect 0xEA
#define ICM20948_USER_CTRL        0x03
#define ICM20948_PWR_MGMT_1       0x06
#define ICM20948_PWR_MGMT_2       0x07
#define ICM20948_INT_STATUS_1     0x1A
#define ICM20948_ACCEL_XOUT_H     0x2D  // ..0x32
#define ICM20948_GYRO_XOUT_H      0x33  // ..0x38
#define ICM20948_TEMP_OUT_H       0x39  // ..0x3A
#define ICM20948_EXT_SLV_SENS_00  0x3B  // ..0x52

/* ---- Bank 2 ---- (configs & sample rate) */
#define ICM20948_GYRO_SMPLRT_DIV     0x00
#define ICM20948_GYRO_CONFIG_1       0x01
#define ICM20948_ODR_ALIGN_EN        0x09
#define ICM20948_ACCEL_SMPLRT_DIV_1  0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2  0x11
#define ICM20948_ACCEL_CONFIG        0x14
#define ICM20948_TEMP_CONFIG         0x53

/* ---- Bank 3 ---- (I2C master to AK09916) */
#define ICM20948_I2C_MST_ODR_CONFIG  0x00
#define ICM20948_I2C_MST_CTRL        0x01
#define ICM20948_I2C_SLV0_ADDR       0x03
#define ICM20948_I2C_SLV0_REG        0x04
#define ICM20948_I2C_SLV0_CTRL       0x05
#define ICM20948_I2C_SLV0_DO         0x06
#define ICM20948_I2C_SLV4_ADDR       0x13
#define ICM20948_I2C_SLV4_REG        0x14
#define ICM20948_I2C_SLV4_CTRL       0x15
#define ICM20948_I2C_SLV4_DO         0x16

/* AK09916 (internal magnetometer) */
#define AK09916_I2C_ADDR         0x0C
#define AK09916_REG_WIA2         0x01 /* expect 0x09 */
#define AK09916_REG_ST1          0x10
#define AK09916_REG_HXL          0x11 /* HXL..HZH = 0x11..0x16, 16-bit little-endian */
#define AK09916_REG_ST2          0x18
#define AK09916_REG_CNTL2        0x31
#define AK09916_REG_CNTL3        0x32
#define AK09916_MODE_CONT_100HZ  0x08
#define AK09916_SOFT_RESET       0x01

/* public I2C handle (use I2C1 per your pinout PB6/PB7) */
extern I2C_HandleTypeDef hi2c1;

/* SI output */
typedef struct __attribute__((packed)) {
    float ax, ay, az;      /* m/s^2 */
    float gx, gy, gz;      /* rad/s */
    float mx, my, mz;      /* tesla */
    float temp_c;          /* °C */
} imu20948_raw_t;

/* API */
bool icm20948_begin(void);
bool icm20948_read_all(imu20948_raw_t* out);      /* one coherent sample */
