/**
 * @file packet_reports.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 所有串口通信数据回报
 * @version 0.1
 * @date 2023-05-31
 *
 * @copyright Copyright (c) 2023
 *
 */
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

typedef struct  {
    uint8_t key_id;
    uint8_t event;
} PacketReportKeyEventTypeDef;


typedef struct {
    uint16_t buttons;
	uint8_t hat;
    int8_t lx;
    int8_t ly;
    int8_t rx;
    int8_t ry;
} PacketReportGamepadTypeDef;
	

#pragma pack()
