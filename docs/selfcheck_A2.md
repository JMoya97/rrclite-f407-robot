# Self-check A2 – Transport & Reliability Hooks

## 1. Transport pre-queue validation
- **`Hiwonder/Portings/packet_porting.c:31-56` – `rrc_transport_send()`**
  - `rrc_func_is_supported(func)` rejects unknown function IDs before enqueue (`lines 33-35`).
  - `rrc_sub_is_supported(func, sub)` ensures the subcommand is registered (`lines 37-39`).
  - `rrc_payload_len_valid_for(func, sub, len)` performs the `(func,sub)→max_len` lookup and length clamp (`lines 41-42`).
  - Null-payload guard protects zero-length copies before handing the frame to `packet_transmit()` (`lines 45-56`).
- **Lookup table backing:** `Core/Src/rrclite_packets.c:23-74` defines `g_sys_subs`, `g_motor_subs`, `g_io_subs`, and `g_imu_subs`, which feed the helper used above.

## 2. Auto-reinit / error reporting paths
- **Motors (PWM open-loop):**
  - `Hiwonder/System/packet_handle.c:521-588` parses optional `txid`, emits `SYS/0xEE` on bad length or hardware apply failure, stubs `motor_pwm_try_reinit()`, and sends `SYS/0xEF` once the fault clears.
  - Reinit hook (`Hiwonder/System/packet_handle.c:49-53`) currently returns `true` immediately; no backoff is implemented yet *(TODO noted below).* 
- **IMU:** Streaming/one-shot helpers in `Hiwonder/Portings/imu_porting.c` do not emit `SYS/0xEE` / `SYS/0xEF` on sensor failures; retries simply abort the frame. *(TODO)*
- **WS2812 LED / bus-servo steering / buzzer:** Command paths lack explicit error → reinit → recovered emission. *(TODO)*

## 3. Motor failsafe watchdog
- State is kept in `Hiwonder/System/packet_handle.c:25-27` (`rrc_motor_failsafe_timeout_ms`, `rrc_motor_last_cmd_ms`).
- PWM handler refreshes `rrc_motor_last_cmd_ms` whenever a command applies successfully (`lines 547-579`).
- `TIM7_IRQHandler` performs the timeout check and ramps both PWM targets to zero without sending extra ACKs (`Core/Src/stm32f4xx_it.c:438-457`).

## 4. UART baud switching safety
- **Handler:** `Hiwonder/System/packet_handle.c:829-868` parses `SYS/0xC0`, validates baud via `rrc_uart_baud_is_supported()`, sends the ACK (`lines 855-866`), then calls the delayed apply helper.
- **Delayed apply:** `Hiwonder/System/packet_handle.c:33-48` (`rrc_uart_apply_with_delay`) waits `apply_after_ms`, reconfigures USART1, and emits `SYS/0xEE` if the reinit fails.
- **Accepted rates / ACK layout:** `Core/Inc/rrclite_packets.h:103-115` declares the only legal baud values (115200, 1 000 000) and the `{txid, baud_le, apply_after_ms_le}` ACK structure. Runtime validation lives in `Core/Src/rrclite_packets.c:286-287`.

## TO FIX
- Implement real recovery/backoff inside `motor_pwm_try_reinit()` and log the retry cadence.
- Add `SYS/0xEE`/`0xEF` emission with backoff for IMU read failures in `imu_task_entry()` / `imu0_read_sample()`.
- Extend LED, bus-servo, and buzzer command paths to surface IO faults via `rrc_send_err()` and `rrc_send_recovered()`.
