# Sensor/Actuator Specific Self-Check (Goal A3)

## Encoders
- Wrap constant: `MOTOR_JGA27_TICKS_PER_CIRCLE` defined as `1040.0f` in `Hiwonder/Portings/motors_param.h` (lines 31–33).
- One-shot path: `encoders_read_once_and_report()` in `Hiwonder/Portings/encoders_porting.c` (lines 79–85) reads TIM5/TIM2 directly when answering SYS/0x90.
- Streaming path: `encoders_timer7_cb()` (lines 43–52) latches the same TIM5/TIM2 counters into `enc_c1/enc_c2`, and `encoders_task_entry()` (lines 92–110) builds sequenced frames from those latched values. Both paths derive from the shared hardware counters; streaming simply uses the periodic latch.

## IMU dual-source (FUNC 0x07)
- Default primary: `g_imu_primary` is a zero-initialised static in `Hiwonder/System/packet_handle.c` (lines 54–58), so the firmware boots with source 0 (ICM-20948) selected.
- WHOAMI/Status: `imu_emit_whoami()` in `Hiwonder/Portings/imu_porting.c` (lines 360–388) returns the cached `imu0_whoami` value (initialised to `0xEA` at line 22) for the primary sensor and `imu1_whoami` for the onboard device.
- ONE_SHOT/STREAM packing: `imu0_read_sample()` (lines 186–210) fills accelerometer/gyro/magnetometer fields from `icm20948_read_all()`, while `imu1_read_sample()` (lines 213–236) provides accelerometer/gyro data and explicitly zeroes `mx/my/mz` because the onboard unit lacks a magnetometer. `imu_task_entry()` (lines 240–305) increments `imu0_seq` and forwards those samples into stream frames.
- Preset setters: `packet_imu_handle()` stores `g_imu_preset[source_id]` (lines 1188–1223) but does not program ODR/LPF registers on either sensor. **TO FIX: program preset values into the hardware after updating the shadow array.**
- Bias setters: `packet_imu_handle()` copies payloads into `g_imu_bias[]` (lines 1226–1261); neither `imu0_read_sample()` nor `imu1_read_sample()` subtract these offsets. **TO FIX: apply the stored biases when populating `rrc_imu_sample_t`.**

## Steering / Bus-servo
- Apply call: `packet_serial_servo_handle()` (lines 689–747) iterates servo targets and calls `serial_servo_set_position()` from `Hiwonder/Peripherals/serial_servo.c` (lines 38–49) to send the MOVE_TIME_WRITE frame.
- Recovery hooks: failures schedule `rrc_io_recovery_state_t` via `rrc_io_recovery_schedule()` (lines 703–708), and `rrc_io_recovery_tick()` (lines 249–256) invokes `rrc_steering_try_reinit()` (lines 210–222) to retry and emit SYS/0xEF on success.
- ACK coverage: the bus-servo path currently emits legacy packet reports but does not send a txid-aware ACK. **TO FIX: add `rrc_send_ack()` for steering applies once the command succeeds.**

## LEDs & Buzzer
- LED payload guards: `packet_led_handle()` checks legacy struct sizes and WS2812 lengths before applying (lines 430–534) and rejects out-of-range IDs via SYS/0xEE. After `led_flash()`/`set_rgb_color()` succeed, it sends `rrc_io_led_ack_t` with `txid` (lines 492–517).
- Buzzer payload guards: `packet_buzzer_handle()` validates both the new `{freq,duty,duration}` layout and the legacy struct, clamping duty to ≤100% and rejecting malformed inputs (lines 560–664). ACKs are sent immediately after a successful `buzzer_didi()`/`buzzer_off()` call (lines 617–664).

## TO FIX
- Program IMU preset selections into device registers instead of only caching them (`Hiwonder/System/packet_handle.c`, lines 1188–1223).
- Apply stored IMU biases when packing one-shot and stream samples (`Hiwonder/Portings/imu_porting.c`, lines 186–236).
- Add a txid-aware ACK to the bus-servo actuator path once `serial_servo_set_position()` succeeds (`Hiwonder/System/packet_handle.c`, lines 689–747).
