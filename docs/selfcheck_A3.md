# Sensor/Actuator Specific Self-Check (Goal A3)

## Encoders
- Wrap constant: `MOTOR_JGA27_TICKS_PER_CIRCLE` defined as `1040.0f` in `Hiwonder/Portings/motors_param.h` (lines 31–33) and referenced by the encoder helpers.
- One-shot path: `encoders_read_once_and_report()` in `Hiwonder/Portings/encoders_porting.c` (lines 79–88) latches TIM5/TIM2 counts and emits ENC/0x20.
- Streaming path: `encoders_timer7_cb()` (lines 43–57) captures the same counters into `enc_c1..enc_c4`, and `encoders_task_entry()` (lines 96–120) wraps them into ENC/0x11 sequenced frames. Both paths use the same latched source data.

## IMU dual-source (FUNC 0x23)
- Default primary: `g_imu_primary` is zero-initialised in `Hiwonder/System/packet_handle.c` (lines 54–59), so source 0 (ICM-20948) is selected on boot.
- WHOAMI/Status: `imu_emit_whoami()` in `Hiwonder/Portings/imu_porting.c` (lines 342–380) reports cached `imu0_whoami` (`0xEA`) for the primary and the onboard value for source 1.
- ONE_SHOT/STREAM packing: `imu0_read_sample()` (lines 192–226) and `imu1_read_sample()` (lines 229–266) subtract `g_imu_bias[source]` for accel/gyro/mag components before returning samples; `imu_task_entry()` (lines 289–360) reuses those corrected structs when publishing stream frames.
- Preset setters: `icm20948_configure_preset()` in `Hiwonder/Peripherals/icm20948.c` (lines 175–236) writes gyro/accel DLPF and sample-rate registers for presets 0/1/2. `packet_imu_handle()` calls `icm20948_apply_preset()` for source 0 before ACKing (lines 1189–1235).
- Bias setters: Stored in `g_imu_bias[]` and applied inside the readers above; one-shot and stream payloads now emit bias-corrected values.

## Steering / Bus-servo
- Apply call: `packet_serial_servo_handle()` in `Hiwonder/System/packet_handle.c` (lines 738–812) iterates targets and calls `serial_servo_set_position()`.
- ACK coverage: On success, the handler builds `rrc_steer_ack_t` and calls `rrc_send_ack(RRCV2_FUNC_STEER, RRC_STEER_ACK, …)` (lines 798–805), echoing txid and the number of accepted targets. The validator row for sub 0x03 lives in `Core/Src/rrclite_packets.c` (lines 60–83).
- Recovery hooks: Failures schedule `rrc_io_recovery_state_t` via `rrc_io_recovery_schedule()` (lines 780–787), and `rrc_io_recovery_service_one()` drives retries before emitting SYS/0xEF on success.

## LEDs & Buzzer
- LED payload guards: `packet_led_handle()` (lines 430–540) checks legacy vs WS2812 lengths, rejects invalid IDs, and after successful apply sends `rrc_send_ack()` with `rrc_led_ack_t` on `RRC_LED_ACK` (lines 503–516).
- Buzzer payload guards: `packet_buzzer_handle()` (lines 560–676) validates `{freq,duty,duration}` or legacy payloads, clamps duty ≤100 %, and ACKs via `rrc_buzz_ack_t` on `RRC_BUZZ_ACK` after latching the command (lines 628–670).

**Status: A3 green.**
