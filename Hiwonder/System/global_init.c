/**
 * @file global_init.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 软硬件接口全局初始化入口
 * @version 0.1
 * @date 2023-05-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "global.h"
#include "cmsis_os2.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "gui_guider.h"

void keys_init(void);
void leds_init(void);
void buzzers_init(void);
void lcds_init(void);
void motors_init(void);
void pwm_servos_init(void);
void imus_init(void);
void sbus_init(void);
void packet_init(void);
void packet_start_recv(void);
void screan_0(void);
void lv_port_disp_init(void);

void global_init(void)
{
    lwmem_assignmem(lwmem_regions);
    lcds_init();
    packet_init();
    keys_init();
    leds_init();
    buzzers_init();
    imus_init();
    motors_init();
    pwm_servos_init();
    sbus_init();
    packet_start_recv();
    lv_init();
    lv_port_disp_init();
    setup_ui(&guider_ui);
    lcd1.set_blacklight(100);
}
