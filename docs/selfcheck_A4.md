# A4 Self-Check – RTOS/perf layout, ISR weight, pacing & caps

## Task inventory

| Task name | Priority | Stack (bytes) | Source | Responsibilities |
| --- | --- | --- | --- | --- |
| `defaultTask` | Below normal | 1024 | `Core/Src/freertos.c` | Auto-generated CMSIS idle helper, no device work.【F:Core/Src/freertos.c†L54-L60】 |
| `imu_task` | Normal | 1024 | `Core/Src/freertos.c`, `Hiwonder/Portings/imu_porting.c` | Waits on the TIM6 semaphore, applies recovery tick bookkeeping, and publishes IMU stream frames.【F:Core/Src/freertos.c†L61-L67】【F:Hiwonder/Portings/imu_porting.c†L255-L317】 |
| `encoder_task` | Normal | 512 | `Core/Src/freertos.c`, `Hiwonder/Portings/encoders_porting.c` | Consumes the encoder stream semaphore latched by TIM7 and emits sequenced frames after checking TX backpressure.【F:Core/Src/freertos.c†L68-L74】【F:Hiwonder/Portings/encoders_porting.c†L35-L110】 |
| `packet_tx_task` | Above normal | 1024 | `Core/Src/freertos.c`, `Hiwonder/Portings/packet_porting.c` | Dequeues transport frames, starts UART DMA, and maintains the transmit idle semaphore.【F:Core/Src/freertos.c†L75-L81】【F:Hiwonder/Portings/packet_porting.c†L146-L176】 |
| `packet_rx_task` | Above normal | 1024 | `Core/Src/freertos.c`, `Hiwonder/Portings/packet_porting.c` | Waits on the RX-not-empty semaphore and runs the frame parser in task context.【F:Core/Src/freertos.c†L82-L88】【F:Hiwonder/Portings/packet_porting.c†L146-L155】 |
| `app_task` | Normal | 2048 | `Core/Src/freertos.c`, `Hiwonder/System/app.c` | Performs bring-up (motors/buttons/streams), starts timers, and polls `rrc_recovery_service()` every 10 ms to run deferred reinitialisation outside ISRs.【F:Core/Src/freertos.c†L98-L104】【F:Hiwonder/System/app.c†L34-L107】 |
| `imu_task`/`encoder_task`/`packet_*` are the only threads that touch transport queues; device actuation stays in their respective workers or timers. GUI and OLED tasks are compiled out in the slim build (`RRC_KEEP_GUI/OLED=0`) so no extra stacks are allocated.【F:Core/Inc/rrclite_config.h†L21-L28】

## ISR workload (bookkeeping only)

- **TIM6_DAC_IRQHandler** only clears the update flag and releases the IMU semaphore—no sensor reads or packet work in the interrupt.【F:Core/Src/stm32f4xx_it.c†L408-L422】
- **TIM7_IRQHandler** latches encoder counts, enforces the failsafe ramp, and *only* schedules recovery ticks; the actual retries and SYS/0xEF emission happen in `rrc_recovery_service()` that `app_task` calls in thread context.【F:Core/Src/stm32f4xx_it.c†L433-L464】【F:Hiwonder/System/packet_handle.c†L99-L132】【F:Hiwonder/System/packet_handle.c†L263-L305】【F:Hiwonder/System/app.c†L87-L107】
- **USART1_IRQHandler** simply unlocks the UART state and defers to the HAL IRQ handler; DMA completions drain the queue in `packet_tx_task` instead of the ISR.【F:Core/Src/stm32f4xx_it.c†L254-L265】【F:Hiwonder/Portings/packet_porting.c†L157-L176】
- **DMA stream ISRs** (RX/TX for USART/I2C/SPI) call the HAL DMA helper and return immediately—no application logic runs in DMA interrupt context.【F:Core/Src/stm32f4xx_it.c†L183-L194】
- **EXTI15_10_IRQHandler** now only clears the IMU GPIO interrupt (the old semaphore release is disabled), avoiding additional work in the line interrupt.【F:Core/Src/stm32f4xx_it.c†L270-L283】

## Stream pacing & caps

- **IMU** – `imu_set_stream()` enforces a minimum `period_ms` of 5 ms (≤200 Hz) and masks unsupported sources; `imu_task_entry()` skips frames when the TX queue has ≥56 entries, incrementing a drop counter instead of blocking.【F:Hiwonder/Portings/imu_porting.c†L345-L359】【F:Hiwonder/Portings/imu_porting.c†L302-L305】
- **Encoders** – `encoders_set_stream()` rounds the requested period to at least one TIM7 tick (10 ms) and resets the sequence; the task checks the same `>=56` TX depth guard before emitting `{seq,c1..c4}` frames.【F:Hiwonder/Portings/encoders_porting.c†L33-L55】【F:Hiwonder/Portings/encoders_porting.c†L96-L110】
- **Buttons** – `buttons_set_stream()` clamps to `BUTTON_TASK_PERIOD` (30 ms ≈33 Hz) and zeroes `buttons_seq`; the periodic callback composes `{seq,mask}` frames on timer wakeups.【F:Hiwonder/Portings/button_porting.c†L33-L53】【F:Hiwonder/Portings/button_porting.c†L86-L107】
- **Battery** – `battery_set_stream()` enforces ≥`BATTERY_TASK_PERIOD` (50 ms = 20 Hz) and maintains `batt_seq`; the battery timer callback packages `{seq,mV}` frames when the period elapses.【F:Hiwonder/System/battery_handle.c†L29-L67】【F:Hiwonder/System/battery_handle.c†L175-L191】
- All pacing timers derive their base periods from `global_conf.h`, keeping buttons at 30 ms and battery at 50 ms.【F:Hiwonder/System/global_conf.h†L56-L60】

## Starvation considerations

- Transport priority favours draining the TX queue: both `packet_tx_task` and `packet_rx_task` run at `osPriorityAboveNormal`, one notch above the producer tasks at `osPriorityNormal`.【F:Core/Src/freertos.c†L61-L88】
- The TX queue holds 64 pending frames; producers drop work once `osMessageQueueGetCount()` reports ≥56 entries (encoders and IMU both use this guard) so actuator ACKs and SYS events continue to flow.【F:Core/Src/freertos.c†L114-L123】【F:Hiwonder/Portings/encoders_porting.c†L99-L110】【F:Hiwonder/Portings/imu_porting.c†L302-L305】
- With the queue guard and higher TX priority, sustained 200 Hz IMU streaming does not starve packet transmission; further load testing can revisit the 56-slot threshold, but no immediate TO-DOs remain.

**Status:** A4 checks complete – no outstanding fixes identified.
