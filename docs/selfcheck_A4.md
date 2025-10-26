# A4 Self-Check – RTOS Tasks, ISR Weight, and Stream Pacing

## Task inventory

| Task name | Priority | Stack (bytes) | Source | Responsibilities |
| --- | --- | --- | --- | --- |
| `defaultTask` | Below normal | 1024 | `Core/Src/freertos.c` | CMSIS auto-generated idle helper; no device work.【F:Core/Src/freertos.c†L54-L60】 |
| `imu_task` | Normal | 1024 | `Hiwonder/Portings/imu_porting.c` | IMU streaming/one-shot publisher driven by TIM6 semaphore.【F:Core/Src/freertos.c†L61-L67】【F:Hiwonder/Portings/imu_porting.c†L250-L307】 |
| `encoder_task` | Normal | 512 | `Hiwonder/Portings/encoders_porting.c` | Encoder stream worker waiting on TIM7 semaphore.【F:Core/Src/freertos.c†L68-L74】【F:Hiwonder/Portings/encoders_porting.c†L12-L110】 |
| `packet_tx_task` | Above normal | 1024 | `Hiwonder/Portings/packet_porting.c` | Serial TX queue pump (DMA submit + idle semaphore).【F:Core/Src/freertos.c†L75-L81】【F:Hiwonder/Portings/packet_porting.c†L176-L210】 |
| `packet_rx_task` | Above normal | 1024 | `Hiwonder/Portings/packet_porting.c` | Serial RX worker parsing frames into handlers.【F:Core/Src/freertos.c†L82-L88】【F:Hiwonder/Portings/packet_porting.c†L160-L175】 |
| `gui_task` | Low | 6000 | `Core/Src/freertos.c` | Weak stub (legacy GUI); no transport duties.【F:Core/Src/freertos.c†L89-L95】【F:Core/Src/freertos.c†L446-L456】 |
| `app_task` | Normal | 2048 | `Hiwonder/System/app.c` | System bring-up, timer start, packet subsystem init, button callbacks.【F:Core/Src/freertos.c†L96-L102】【F:Hiwonder/System/app.c†L24-L96】 |
| `oled_task` | Below normal | 1024 | `Hiwonder/System/oled_handle.c` | Display refresh (legacy); weak stub if disabled.【F:Core/Src/freertos.c†L103-L109】【F:Hiwonder/System/oled_handle.c†L16-L64】 |

- `packet_rx_task`/`packet_tx_task` are the only tasks touching the transport queues; device acquisition happens in `imu_task`, `encoder_task`, and the timer callbacks that wake them. GUI/OLED/default tasks remain inert for the slim firmware build.

## Interrupt service routines (lightweight)

- **TIM6_DAC_IRQHandler** – releases the IMU semaphore, no I/O inside the ISR.

  ```c
  if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET &&
      __HAL_TIM_GET_IT_SOURCE(&htim6, TIM_IT_UPDATE) != RESET) {
    __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
    if (IMU_data_readyHandle) {
      osSemaphoreRelease(IMU_data_readyHandle);
    }
  }
  ```
  【F:Core/Src/stm32f4xx_it.c†L411-L422】

- **TIM7_IRQHandler** – clears the update flag, runs failsafe/backoff bookkeeping, no blocking I/O; encoder streaming is deferred to the task via `encoders_timer7_cb()`.

  ```c
  if (__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) != RESET &&
      __HAL_TIM_GET_IT_SOURCE(&htim7, TIM_IT_UPDATE) != RESET) {
    __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
    encoders_timer7_cb();
    const uint32_t now_ms = HAL_GetTick();
    if (failsafe_timeout > 0U &&
        (uint32_t)(now_ms - rrc_motor_last_cmd_ms) >= (uint32_t)failsafe_timeout) {
      for (int i = 0; i < 2; ++i) motors_pwm_target[i] = 0;
    }
    rrc_motor_recovery_tick(now_ms);
    rrc_imu_recovery_tick(now_ms);
    rrc_io_recovery_tick(now_ms);
  }
  ```
  【F:Core/Src/stm32f4xx_it.c†L433-L465】

- **USART1_IRQHandler** – unlocks the HAL state and hands control to the HAL IRQ handler (DMA already manages buffers).

  ```c
  __HAL_UNLOCK(&huart1);
  HAL_UART_IRQHandler(&huart1);
  ```
  【F:Core/Src/stm32f4xx_it.c†L256-L264】

- **DMA handlers** – delegate to HAL DMA IRQ (no application code).

  ```c
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
  HAL_DMA_IRQHandler(&hdma_i2c2_tx);
  ```
  【F:Core/Src/stm32f4xx_it.c†L183-L361】【F:Core/Src/stm32f4xx_it.c†L532-L568】

- **EXTI15_10_IRQHandler** – (legacy) only clears the GPIO interrupt and optionally releases the IMU semaphore; no blocking work occurs here.

  ```c
  if(__HAL_GPIO_EXTI_GET_IT(IMU_ITR_Pin) != RESET) {
    __HAL_GPIO_EXTI_CLEAR_IT(IMU_ITR_Pin);
    osSemaphoreRelease(IMU_data_readyHandle);
  }
  ```
  【F:Core/Src/stm32f4xx_it.c†L270-L281】

## Stream pacing and caps

- **IMU streaming** – `imu_set_stream()` enforces `period_ms >= 5`, so the primary ICM-20948 stream tops out at 200 Hz. TIM6 generates the pacing semaphore, and `imu_task_entry()` drops frames while an error backoff is active.【F:Hiwonder/Portings/imu_porting.c†L335-L376】【F:Hiwonder/Portings/imu_porting.c†L250-L307】

- **Encoder streaming** – TIM7 ISR latches counts and releases `s_enc_stream_sem`; `encoders_set_stream()` rounds the host request to ≥1 tick of the 10 ms TIM7 period (≤100 Hz). A queue depth guard skips publishes when ≥56 items are pending to protect the TX path.【F:Hiwonder/Portings/encoders_porting.c†L10-L110】

- **Buttons** – `buttons_set_stream()` clamps the period to `BUTTON_TASK_PERIOD` (30 ms, ≈33 Hz). The CMSIS timer callback increments `buttons_seq` and sends frames via the transport helper.【F:Hiwonder/Portings/button_porting.c†L13-L107】【F:Hiwonder/System/global_conf.h†L56-L60】

- **Battery** – `battery_set_stream()` enforces `period_ms >= BATTERY_TASK_PERIOD` (50 ms, 20 Hz). The heartbeat timer reuses that guard while reporting `{seq, millivolts}` frames.【F:Hiwonder/System/battery_handle.c†L21-L191】【F:Hiwonder/System/global_conf.h†L56-L60】

All stream producers rely on timer-driven pacing and explicit minimum periods, so no path can exceed the documented caps (IMU ≤200 Hz, encoders ≤100 Hz, buttons/battery tens of Hz).

## Starvation considerations

- Transport priorities favour draining the TX queue: `packet_tx_task` runs at `osPriorityAboveNormal`, one notch above `imu_task`/`encoder_task` at `osPriorityNormal`, so completed frames are flushed before producers resume.【F:Core/Src/freertos.c†L61-L88】

- The TX queue has 64 slots, and encoders guard against overfilling by checking `osMessageQueueGetCount(packet_tx_queueHandle) >= 56` before enqueuing a frame.【F:Core/Src/freertos.c†L110-L120】【F:Hiwonder/Portings/encoders_porting.c†L95-L110】

- `imu_task_entry()` currently relies on the higher TX priority (and optional frame ACK disable) rather than an explicit queue-count guard; at the 200 Hz cap the queue still drains promptly, but long ACK bursts could pressure the depth. No action required now, just noted for future load testing.

**Status:** All A4 checks complete; no additional TODOs discovered.

