# Self-check A2 – Transport & Reliability Hooks

## 1. Transport pre-queue validation
- **Hiwonder/Portings/packet_porting.c:31-56 — `rrc_transport_send()`**
  - Function gate: `if (!rrc_func_is_supported(func)) { return false; }` (lines 33-35) rejects unknown function IDs before any frame is built.
  - Subcommand gate: `if (!rrc_sub_is_supported(func, sub)) { return false; }` (lines 37-39) enforces that the `(func, sub)` pair is registered.
  - Length guard: `if (!rrc_payload_len_valid_for(func, sub, len)) { return false; }` (lines 41-42) performs the lookup against the `(func,sub)→max_len` tables in `Core/Src/rrclite_packets.c:32-219`.
  - Payload pointer guard: `if (len > 0U && payload == NULL) { return false; }` (lines 49-52) blocks null buffers.

## 2. Backoff-driven recovery paths

### Motor PWM (FUNC=0x03, SUB=0x10)
- **First ERR emission:** `Hiwonder/System/packet_handle.c:984-995` inside `packet_motor_handle()` detects a failed apply, seeds backoff, and raises `rrc_send_err(RRC_FUNC_MOTOR, RRC_MOTOR_PWM_SET, RRC_SYS_ERR_IO_FAIL, …)` once per episode.
- **Backoff parameters:** `motor_backoff_ensure_init()` (`packet_handle.c:98-105`) initialises `g_motor_backoff` with `initial_ms=50`, `factor=3.0f`, `max_ms=1000`.
- **Retry scheduling state:** `g_motor_err_active` / `g_motor_retry_due_ms` (declared at `packet_handle.c:36-38`) gate retries and store the next attempt deadline via `rrc_backoff_next()`.
- **RECOVERED emission:** `rrc_motor_recovery_tick()` (`packet_handle.c:107-127`) runs from `TIM7_IRQHandler` and calls `rrc_send_recovered(RRC_FUNC_MOTOR, RRC_MOTOR_PWM_SET, RRC_SYS_ERR_IO_FAIL, …)` after a successful retry.

### IMU (FUNC=0x07 — stream & one-shot)
- **First ERR emission:** `imu_schedule_failure()` (`Hiwonder/Portings/imu_porting.c:61-82`) is invoked from both streaming (`imu_task_entry` loop) and one-shot paths; it sends `rrc_send_err(RRC_FUNC_IMU, origin_sub, err_code, source_id, …)` the first time a source fails and marks `g_imu_err_active[source]`.
- **Backoff parameters:** `imu_backoff_init_once()` (`imu_porting.c:48-58`) initialises each `g_imu_backoff[source]` with `initial_ms=50`, `factor=3.0f`, `max_ms=1000`.
- **Retry scheduling state:** `g_imu_retry_due_ms[source]` (declared at `imu_porting.c:32`) stores the next retry instant; `imu_schedule_failure()` updates it via `rrc_backoff_next()`.
- **RECOVERED emission:** `imu_clear_error()` (`imu_porting.c:90-104`) and `rrc_imu_recovery_tick()` (`imu_porting.c:391-408`) emit `rrc_send_recovered(RRC_FUNC_IMU, g_imu_last_origin_sub[source], RRC_SYS_ERR_IO_FAIL, …)` when the retry succeeds.

### LEDs (FUNC=0x04, SUB=0x20)
- **First ERR emission:** On apply failure, `packet_led_handle()` (`packet_handle.c:478-484`) calls `rrc_io_recovery_schedule(&g_led_recovery, …)`, which sends `rrc_send_err(RRC_FUNC_IO, RRC_IO_LED_SET, RRC_SYS_ERR_IO_FAIL, detail, …)` the first time (`packet_handle.c:163-166`).
- **Backoff parameters:** `rrc_io_recovery_schedule()` initialises `g_led_recovery.backoff` with `initial_ms=50`, `factor=3.0f`, `max_ms=1000` (`packet_handle.c:152-170`).
- **Retry scheduling state:** `g_led_recovery.retry_due_ms` (`packet_handle.c:142-170`) holds the next retry deadline; retries are attempted in `rrc_io_recovery_tick_one()`.
- **RECOVERED emission:** Successful retries through `rrc_led_try_reinit()` trigger `rrc_send_recovered(RRC_FUNC_IO, RRC_IO_LED_SET, …)` in `rrc_io_recovery_tick_one()` (`packet_handle.c:225-240`), while direct successes clear the state via `rrc_io_recovery_clear()` (`packet_handle.c:173-186`).

### Buzzer (FUNC=0x04, SUB=0x21)
- **First ERR emission:** `packet_buzzer_handle()` schedules recovery on apply failure (`packet_handle.c:600-639`), leading to `rrc_send_err(RRC_FUNC_IO, RRC_IO_BUZZER_SET, …)` inside `rrc_io_recovery_schedule()`.
- **Backoff parameters:** Same helper as LEDs — `initial_ms=50`, `factor=3.0f`, `max_ms=1000` (`packet_handle.c:152-170`).
- **Retry scheduling state:** `g_buzzer_recovery.retry_due_ms` plus `g_buzzer_recovery.err_active` (declared at `packet_handle.c:69-70`) track retry cadence.
- **RECOVERED emission:** `rrc_io_recovery_tick_one()` (`packet_handle.c:225-240`) calls `rrc_send_recovered(RRC_FUNC_IO, RRC_IO_BUZZER_SET, …)` after `rrc_buzzer_try_reinit()` succeeds; immediate clears use `rrc_io_recovery_clear()`.

### Steering / Bus-servo (FUNC=0x06 legacy, SUB=0x01)
- **First ERR emission:** `packet_serial_servo_handle()` routes failed `serial_servo_set_position()` attempts to `rrc_io_recovery_schedule(&g_steering_recovery, PACKET_FUNC_BUS_SERVO, 0x01U, …)` (`packet_handle.c:700-705`), which in turn emits `rrc_send_err(PACKET_FUNC_BUS_SERVO, 0x01, …)` once (`packet_handle.c:163-166`).
- **Backoff parameters:** `g_steering_recovery.backoff` is initialised through the same helper with `initial_ms=50`, `factor=3.0f`, `max_ms=1000` (`packet_handle.c:152-170`).
- **Retry scheduling state:** `g_steering_recovery.retry_due_ms` (struct member) stores the next retry time, driven by `rrc_io_recovery_tick_one()` (`packet_handle.c:225-240`).
- **RECOVERED emission:** When `rrc_steering_try_reinit()` (`packet_handle.c:213-223`) succeeds, `rrc_io_recovery_tick_one()` issues `rrc_send_recovered(PACKET_FUNC_BUS_SERVO, 0x01, …)`; direct success paths also call `rrc_io_recovery_clear()` (`packet_handle.c:711-713`).
