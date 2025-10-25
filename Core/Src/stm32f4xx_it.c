/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "global.h"
#include "tim.h"
#include "serial_servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim14;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1) {
    }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
    extern EncoderMotorObjectTypeDef *motors[2];
    if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
            --motors[1]->overflow_num;
        } else {
            ++motors[1]->overflow_num;
        }
        //printf("motor2, counts:%d", motors[1]->overflow_num * 60000 + __HAL_TIM_GetCounter(&htim2));
    }
  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
    extern EncoderMotorObjectTypeDef *motors[2];
    if(__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5)) {
            --motors[0]->overflow_num;
        } else {
            ++motors[0]->overflow_num;
        }
        //printf("motor1, counts:%d", motors[0]->overflow_num * 60000 + __HAL_TIM_GetCounter(&htim5));
    }
  /* USER CODE END TIM5_IRQn 0 */
  /* USER CODE BEGIN TIM5_IRQn 1 */
  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
    __HAL_UNLOCK(&huart1); /* Unlock to avoid deadlock */
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	#if 0  // IMU pacing now done by TIM6 @ 100 Hz
		extern osSemaphoreId_t IMU_data_readyHandle;
		if(__HAL_GPIO_EXTI_GET_IT(IMU_ITR_Pin) != RESET) {
			__HAL_GPIO_EXTI_CLEAR_IT(IMU_ITR_Pin);
			osSemaphoreRelease(IMU_data_readyHandle);
		}
	#else
		if(__HAL_GPIO_EXTI_GET_IT(IMU_ITR_Pin) != RESET) {
			__HAL_GPIO_EXTI_CLEAR_IT(IMU_ITR_Pin);
		}
	#endif
  /* USER CODE END EXTI15_10_IRQn 0 */
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */
    if( __HAL_TIM_GET_FLAG(&htim12, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_UPDATE);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    }
    if(__HAL_TIM_GET_FLAG(&htim12, TIM_FLAG_CC1) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_CC1);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    }
  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */
    extern PWMServoObjectTypeDef *pwm_servos[4];
    static uint32_t pwm_servo_index = 0;

    if(__HAL_TIM_GET_FLAG(&htim13, TIM_FLAG_CC1) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&htim13, TIM_FLAG_CC1);
        pwm_servos[pwm_servo_index]->write_pin(0);
        pwm_servo_index = pwm_servo_index == 3 ? 0 : pwm_servo_index + 1;
    }
    if(__HAL_TIM_GET_FLAG(&htim13, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&htim13, TIM_FLAG_UPDATE);
        pwm_servos[pwm_servo_index]->write_pin(1);
        pwm_servo_duty_compare(pwm_servos[pwm_servo_index]);
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, pwm_servos[pwm_servo_index]->duty_raw);
    }
  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream7 global interrupt.
  */
void DMA1_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream7_IRQn 0 */

  /* USER CODE END DMA1_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_tx);
  /* USER CODE BEGIN DMA1_Stream7_IRQn 1 */

  /* USER CODE END DMA1_Stream7_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
    extern SerialServoControllerTypeDef serial_servo_controller;
    extern osSemaphoreId_t serial_servo_rx_completeHandle;

    if(__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE) != RESET) {
        __HAL_UART_CLEAR_FLAG(&huart5, UART_FLAG_RXNE);
        if(0 == serial_servo_rx_handler(&serial_servo_controller, (uint8_t)(huart5.Instance->DR & (uint8_t)0x00FF))) {
            osSemaphoreRelease(serial_servo_rx_completeHandle);
        }
    }
	
    if(__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TC) != RESET) {
		__HAL_UART_CLEAR_FLAG(&huart5, UART_FLAG_TC);
        if(serial_servo_controller.tx_only) {
            osSemaphoreRelease(serial_servo_rx_completeHandle);
        }else{
//			HAL_GPIO_WritePin(SERIAL_SERVO_RX_EN_GPIO_Port, SERIAL_SERVO_RX_EN_Pin, GPIO_PIN_RESET);  /* Switch to receive mode */
//			HAL_GPIO_WritePin(SERIAL_SERVO_TX_EN_GPIO_Port, SERIAL_SERVO_TX_EN_Pin, GPIO_PIN_SET);
            HAL_HalfDuplex_EnableReceiver(&huart5);
		}
		__HAL_UART_DISABLE_IT(&huart5, UART_IT_TC);
		return;
    }
    
    if(__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TXE) != RESET) {
        __HAL_UART_CLEAR_FLAG(&huart5, UART_FLAG_TXE);
        if(serial_servo_controller.tx_byte_index < serial_servo_controller.tx_frame.elements.length + 3) {  /* Check whether all data has been transmitted */
            huart5.Instance->DR = ((uint8_t*)(&serial_servo_controller.tx_frame))[serial_servo_controller.tx_byte_index++]; /* Continue sending the next byte */
        } else {
            __HAL_UART_DISABLE_IT(&huart5, UART_IT_TXE);
        }
    }
  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */
    
  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt (shared with DAC).
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET &&
      __HAL_TIM_GET_IT_SOURCE(&htim6, TIM_IT_UPDATE) != RESET)
  {
    __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
    extern osSemaphoreId_t IMU_data_readyHandle;
    if (IMU_data_readyHandle) {
      osSemaphoreRelease(IMU_data_readyHandle);
    }
  }
  /* USER CODE END TIM6_DAC_IRQn 0 */
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
  /* USER CODE END TIM6_DAC_IRQn 1 */
}



/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
    /* USER CODE BEGIN TIM7_IRQn 0 */
    extern EncoderMotorObjectTypeDef *motors[2];
    extern volatile int motors_pwm_target[2];
    extern volatile int motors_pwm_current[2];
    extern volatile uint16_t rrc_motor_failsafe_timeout_ms;
    extern volatile uint32_t rrc_motor_last_cmd_ms;

    if (__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) != RESET &&
        __HAL_TIM_GET_IT_SOURCE(&htim7, TIM_IT_UPDATE) != RESET) {

        __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);

        // latch counters & schedule encoder stream (no TX in IRQ)
        extern void encoders_timer7_cb(void);
        encoders_timer7_cb();

        const uint32_t now_ms = HAL_GetTick();
        const uint16_t failsafe_timeout = rrc_motor_failsafe_timeout_ms;
        if (failsafe_timeout > 0U) {
            if ((uint32_t)(now_ms - rrc_motor_last_cmd_ms) >=
                (uint32_t)failsafe_timeout) {
                for (int i = 0; i < 2; ++i) {
                    motors_pwm_target[i] = 0;
                }
            }
        }

        // keep your existing 10 ms math exactly as-is
        uint32_t c0 = __HAL_TIM_GET_COUNTER(&htim5);
        uint32_t c1 = __HAL_TIM_GET_COUNTER(&htim2);

        encoder_update(motors[0], 0.01f, c0);
        encoder_update(motors[1], 0.01f, c1);

    #if ENABLE_MOTOR_PID_LOOP
        for (int i = 0; i < 2; ++i) {
            encoder_motor_control(motors[i], 0.01f);
        }
    #endif

        /* Slew target → current (visible ramp) */
        for (int i = 0; i < 2; ++i) {
            int cur  = motors_pwm_current[i];
            int tgt  = motors_pwm_target[i];

            /* Read measured speed (rps) from motor object updated by encoder_update() */
            float rps = motors[i]->rps;  /* if you prefer TPS: use motors[i]->tps / 1040.0f */

            /* ---- A) Reverse-gate: if direction flips while still spinning, aim for zero first ---- */
            if (((cur > 0 && tgt < 0) || (cur < 0 && tgt > 0)) &&
                ((rps > REVERSE_GATE_RPS_THRESH) || (rps < -REVERSE_GATE_RPS_THRESH))) {
                tgt = 0;  /* temporarily force zero until the wheel is nearly stopped */
            }

            /* ---- B) Gentle hold at zero: when target==0 but wheel is coasting, oppose slightly ---- */
            if (tgt == 0) {
                float mag = (rps >= 0.0f) ? rps : -rps;
                if (mag > HOLD_RPS_DEADBAND) {
                    int hold = PWM_COAST_HOLD_PWM;
                    /* if we're already near idle PWM, nudge with a small opposite value */
                    if (cur > -NORMAL_IDLE_DEADBAND && cur < NORMAL_IDLE_DEADBAND) {
                        tgt = (rps > 0.0f) ? -hold : hold;
                    }
                }
            }

            /* ---- C) Slew toward (possibly adjusted) target ---- */
            int diff = tgt - cur;
            if (diff > 0) {
                int step = PWM_SLEW_UP_PER_TICK;
                if (diff < step) step = diff;
                cur += step;
            } else if (diff < 0) {
                int step = PWM_SLEW_DOWN_PER_TICK;
                if (-diff < step) step = -diff;
                cur -= step;
            }

            /* clamp just in case */
            if (cur > 1000) cur = 1000;
            if (cur < -1000) cur = -1000;

            if (cur != motors_pwm_current[i]) {
                motors_pwm_current[i] = cur;
                motors[i]->set_pulse(motors[i], cur);
            }
        }
    }
    /* USER CODE END TIM7_IRQn 0 */
    /* USER CODE BEGIN TIM7_IRQn 1 */
    /* USER CODE END TIM7_IRQn 1 */
}
/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
