#include "stm32f4xx.h"
#include "tim.h"
#include "cmsis_os2.h"

#include "global_conf.h"
#include "lwmem_porting.h"

#include "buzzer.h"

BuzzerObjectTypeDef *buzzers[1];
static osMessageQueueId_t buzzer1_ctrl_ququeHandle; /* Buzzer control queue handle */

static void buzzer1_set_pwm(BuzzerObjectTypeDef *self, uint32_t freq);         /* PWM configuration hook */
static int put_ctrl_block(BuzzerObjectTypeDef *self, BuzzerCtrlTypeDef *p);   /* Enqueue control block */
static int get_ctrl_block(BuzzerObjectTypeDef *self, BuzzerCtrlTypeDef *p);   /* Dequeue control block */

void buzzers_init(void)
{
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);  /* Drive the buzzer pin low */

        /* Create buzzer control queue */
	const osMessageQueueAttr_t buzzer1_ctrl_quque_attributes = { .name = "buzzer1_ctrl_quque" };
	buzzer1_ctrl_ququeHandle = osMessageQueueNew (5, sizeof(BuzzerCtrlTypeDef), &buzzer1_ctrl_quque_attributes);
	
        /* Instantiate the buzzer object */
    buzzers[0] = LWMEM_CCM_MALLOC(sizeof(BuzzerObjectTypeDef)); 
	buzzer_object_init(buzzers[0]);
	buzzers[0]->id = 1;
    buzzers[0]->set_pwm = buzzer1_set_pwm;
	buzzers[0]->get_ctrl_block = get_ctrl_block;
	buzzers[0]->put_ctrl_block = put_ctrl_block;

        /* Timer parameters are configured through STM32CubeMX */
    __HAL_TIM_SET_COUNTER(&htim12, 0);               /* Reset timer counter */
    __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_UPDATE);  /* Clear update interrupt flag */
    __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_CC1);     /* Clear compare interrupt flag */
    __HAL_TIM_ENABLE_IT(&htim12, TIM_IT_UPDATE);     /* Enable update interrupt */
    __HAL_TIM_ENABLE_IT(&htim12, TIM_IT_CC1);        /* Enable compare interrupt */
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


