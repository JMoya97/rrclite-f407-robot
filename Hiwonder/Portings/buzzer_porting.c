#include "stm32f4xx.h"
#include "tim.h"
#include "cmsis_os2.h"

#include "global_conf.h"
#include "lwmem_porting.h"

#include "buzzer.h"

BuzzerObjectTypeDef *buzzers[1];
static osMessageQueueId_t buzzer1_ctrl_ququeHandle; /* 蜂鸣器控制队列Handle */

static void buzzer1_set_pwm(BuzzerObjectTypeDef *self, uint32_t freq);         /* pwm设置接口 */
static int put_ctrl_block(BuzzerObjectTypeDef *self, BuzzerCtrlTypeDef *p);   /* 控制入队接口 */
static int get_ctrl_block(BuzzerObjectTypeDef *self, BuzzerCtrlTypeDef *p);   /* 控制出队接口 */

void buzzers_init(void)
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);  /* 设置蜂鸣器引脚IO为低电平 */
	
	/* 建立蜂鸣器控制队列 */
	const osMessageQueueAttr_t buzzer1_ctrl_quque_attributes = { .name = "buzzer1_ctrl_quque" };
	buzzer1_ctrl_ququeHandle = osMessageQueueNew (5, sizeof(BuzzerCtrlTypeDef), &buzzer1_ctrl_quque_attributes);
	
	/* 建立蜂鸣器对象实例 */
    buzzers[0] = LWMEM_CCM_MALLOC(sizeof(BuzzerObjectTypeDef)); 
	buzzer_object_init(buzzers[0]);
	buzzers[0]->id = 1;
    buzzers[0]->set_pwm = buzzer1_set_pwm;
	buzzers[0]->get_ctrl_block = get_ctrl_block;
	buzzers[0]->put_ctrl_block = put_ctrl_block;

	/* 定时器的各个参数配置有 STM32CubeMX 软件配置生成 */
    __HAL_TIM_SET_COUNTER(&htim12, 0);               /* 清零定时器计数值 */
    __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_UPDATE);  /* 清除定时器更新中断标志 */
    __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_CC1);     /* 清除定时器比较中断标志 */
    __HAL_TIM_ENABLE_IT(&htim12, TIM_IT_UPDATE);     /* 使能定时器更新中断 */
    __HAL_TIM_ENABLE_IT(&htim12, TIM_IT_CC1);        /* 使能定时器比较中断 */
}

void buzzer_timer_callback(void *argument)
{
    buzzer_task_handler(buzzers[0], BUZZER_TASK_PERIOD);
}

static void buzzer1_set_pwm(BuzzerObjectTypeDef *self, uint32_t freq)
{
	freq  = freq > 20000 ? 20000: freq;

    if(freq < 10) {
        __HAL_TIM_DISABLE(&htim12);
        __HAL_TIM_SET_COUNTER(&htim12, 0);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
        return;
    }

    uint32_t period = 100000u / freq;
    uint16_t pulse = period / 2;
    uint32_t counter_period = period - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim12, counter_period);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pulse);
    __HAL_TIM_ENABLE(&htim12);
}

static int put_ctrl_block(BuzzerObjectTypeDef *self, BuzzerCtrlTypeDef *p) {
	return (int)osMessageQueuePut(buzzer1_ctrl_ququeHandle, p, 0, 0);
}

static int get_ctrl_block(BuzzerObjectTypeDef *self, BuzzerCtrlTypeDef *p) {
	return (int)osMessageQueueGet(buzzer1_ctrl_ququeHandle, p, 0, 0);
}


