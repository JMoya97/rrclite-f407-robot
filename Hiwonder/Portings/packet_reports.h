#include <stdint.h>

#pragma pack(1)
typedef union  {
    float array[4];
    struct {
        float w;
        float x;
        float y;
        float z;
    } element;
} PacketReportIMU_Quat_TypeDef;

typedef struct {
	uint8_t sub_cmd;
	uint16_t voltage;
}PacketReportBatteryVoltageTypeDef;

typedef union  {
    struct {
		float accel_array[3];
		float gyro_array[3];
	}array;
    struct {
        struct {
            float x;
            float y;
            float z;
        } accel;
        struct {
            float x;
            float y;
            float z;
        } gyro;
    } element;
} PacketReportIMU_Raw_TypeDef;

typedef struct {
    uint8_t  sub_cmd;
    uint32_t t_ms;
    struct { float x, y, z; } accel;   /* m/s^2 */
    struct { float x, y, z; } gyro;    /* rad/s */
    struct { float x, y, z; } mag;     /* µT */
    float    temp_c;                   /* °C */
} PacketReportIMU_Raw_V2_TypeDef;

typedef struct {
    uint8_t  sub;          /* 0x18 */
    uint32_t t_ms;         /* HAL_GetTick() */
    uint8_t  motor_id;     /* 0..3 (we use 0..1) */
    int16_t  pwm_target;   /* clamped target we accepted */
    int16_t  pwm_applied;  /* current applied (slewed) at ack time */
} PacketReportMotorPwmAck_Single;

typedef struct {
    uint8_t  sub;          /* 0x19 */
    uint32_t t_ms;
    uint8_t  count;        /* number of entries (<=2 for us) */
    struct {
        uint8_t motor_id;
        int16_t pwm_target;
        int16_t pwm_applied;
    } item[2];
} PacketReportMotorPwmAck_Multi;

typedef struct {
    uint8_t  sub;     /* 0x90 (one-shot) or 0x91 (stream) */
    uint32_t t_ms;    /* HAL_GetTick() timestamp */
    uint16_t c1;      /* M1 encoder raw counter (TIM5) */
    uint16_t c2;      /* M2 encoder raw counter (TIM2) */
} EncoderReadReportTypeDef;

typedef struct  {
    uint8_t key_id;
    uint8_t event;
} PacketReportKeyEventTypeDef;

typedef struct {
	uint8_t servo_id;
	uint8_t sub_command;
	uint8_t success;
	uint8_t args[8];
}PacketReportSerialServoTypeDef;

typedef struct {
	uint8_t servo_id;
	uint8_t sub_command;
	uint8_t args[8];
}PacketReportPWMServoTypeDef;
#pragma pack()
