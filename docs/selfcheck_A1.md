# Protocol Surface Self-Check (Goal A1)

## (FUNC, SUB) payload expectations

| FUNC | SUB | Direction | Max payload (bytes) |
| --- | --- | --- | --- |
| SYS (0x00) | 0xA0 Battery one-shot | req | 0 |
| SYS (0x00) | 0xA0 Battery one-shot | resp | 2 |
| SYS (0x00) | 0xA1 Battery stream ctrl | req | 4 |
| SYS (0x00) | 0xA1 Battery stream ctrl | resp (ACK) | 4 |
| SYS (0x00) | 0xA1 Battery stream frame | resp (frame) | 4 |
| SYS (0x00) | 0xB0 Motor failsafe set | req | 3 |
| SYS (0x00) | 0xB0 Motor failsafe set | resp (ACK) | 3 |
| SYS (0x00) | 0xB1 Heartbeat period set | req | 3 |
| SYS (0x00) | 0xB1 Heartbeat period set | resp (ACK) | 3 |
| SYS (0x00) | 0xC0 UART baud set | req | 7 |
| SYS (0x00) | 0xC0 UART baud set | resp (ACK) | 7 |
| SYS (0x00) | 0xC1 UART baud get | req | 0 |
| SYS (0x00) | 0xC1 UART baud get | resp | 4 |
| SYS (0x00) | 0xE0 Ping/Echo | req | ≤254 |
| SYS (0x00) | 0xE0 Ping/Echo | resp | ≤254 |
| SYS (0x00) | 0xEE Error event | event | 9 |
| SYS (0x00) | 0xEF Recovered event | event | 7 |
| SYS (0x00) | 0xF0 Version | req | 0 |
| SYS (0x00) | 0xF0 Version | resp | 4 |
| SYS (0x00) | 0xF1 Capabilities | req | 0 |
| SYS (0x00) | 0xF1 Capabilities | resp | 16 |
| MOTOR (0x03) | 0x10 PWM set (single) | req | 4 |
| MOTOR (0x03) | 0x18 PWM ACK (single) | resp | 6 |
| MOTOR (0x03) | 0x19 PWM ACK (multi) | resp | ≤254 |
| MOTOR (0x03) | 0x90 Encoders one-shot | req | 0 |
| MOTOR (0x03) | 0x90 Encoders one-shot | resp | 10 |
| MOTOR (0x03) | 0x91 Encoders stream ctrl | req | 4 |
| MOTOR (0x03) | 0x91 Encoders stream ctrl | resp (ACK) | 4 |
| MOTOR (0x03) | 0x91 Encoders stream frame | resp (frame) | 10 |
| MOTOR (0x03) | 0x99 Encoder frame ACK | resp | 2 |
| IO (0x04) | 0x20 LEDs set | req | 7 |
| IO (0x04) | 0x20 LEDs set | resp (ACK) | 2 |
| IO (0x04) | 0x21 Buzzer set | req | 5 |
| IO (0x04) | 0x21 Buzzer set | resp (ACK) | 6 |
| IO (0x04) | 0x22 Buttons one-shot | req | 0 |
| IO (0x04) | 0x22 Buttons one-shot | resp | 1 |
| IO (0x04) | 0x23 Buttons stream ctrl | req | 4 |
| IO (0x04) | 0x23 Buttons stream ctrl | resp (ACK) | 4 |
| IO (0x04) | 0x23 Buttons stream frame | resp (frame) | 3 |
| IO (0x04) | 0x29 Buttons frame ACK | resp | 2 |
| IMU (0x07) | 0xA0 IMU one-shot | req | 1 |
| IMU (0x07) | 0xA0 IMU one-shot | resp | 45 |
| IMU (0x07) | 0xA1 IMU stream ctrl | req | 5 |
| IMU (0x07) | 0xA1 IMU stream ctrl | resp (ACK) | 5 |
| IMU (0x07) | 0xA1 IMU stream frame | resp (frame) | 47 |
| IMU (0x07) | 0xA2 Set primary source | req | 2 |
| IMU (0x07) | 0xA2 Set primary source | resp (ACK) | 2 |
| IMU (0x07) | 0xA3 Set preset | req | 3 |
| IMU (0x07) | 0xA3 Set preset | resp (ACK) | 3 |
| IMU (0x07) | 0xA4 Set biases | req | 38 |
| IMU (0x07) | 0xA4 Set biases | resp (ACK) | 2 |
| IMU (0x07) | 0xA5 WHOAMI/Status | req | 1 |
| IMU (0x07) | 0xA5 WHOAMI/Status | resp | 3 |
| IMU (0x07) | 0xA9 IMU frame ACK | resp | 3 |

*Request payload lengths are counted after the sub-command byte; “resp” and “event” rows reflect the validated maxima from `g_*_subs` in `Core/Src/rrclite_packets.c`.*

## Optional txid parsing & echo locations

- Motor PWM set: `packet_motor_handle()` in `Hiwonder/System/packet_handle.c` parses optional `txid` for sub 0x10 and echoes it via `rrc_send_ack()` (lines 520–600 region).
- Encoder stream control: `packet_encoder_handle()` in `Hiwonder/System/packet_handle.c` handles sub 0x91, captures `txid`, and echoes it in the ACK.
- Battery stream control: `packet_battery_limit_handle()` in `Hiwonder/System/packet_handle.c` handles sub 0xA1, reads the optional `txid`, and echoes it in the ACK.
- Button stream control: `packet_pwm_servo_handle()` in `Hiwonder/System/packet_handle.c` handles sub 0x23, reads optional `txid`, and echoes it.
- IMU stream control: `packet_imu_handle()` in `Hiwonder/System/packet_handle.c` handles sub 0xA1, reads optional `txid`, and echoes it through `rrc_send_ack()`.

## Stream sequence counters

- Encoders: `enc_seq` in `Hiwonder/Portings/encoders_porting.c`.
- Battery: `batt_seq` in `Hiwonder/System/battery_handle.c`.
- Buttons: `buttons_seq` in `Hiwonder/Portings/button_porting.c`.
- IMU primary (ICM-20948): `imu0_seq` in `Hiwonder/Portings/imu_porting.c`.
- IMU onboard secondary: no streaming counter is defined (source 1 supports one-shot/WHOAMI only per current implementation).

## Error codes & emission sites

- `rrc_error_code_t` is defined in `Core/Inc/rrclite_packets.h` with values: `InvalidArg (1)`, `Busy (2)`, `Timeout (3)`, `IOFail (4)`, `NotReady (5)`, `NoDevice (6)`, `CRCFail (7)`, `Unsupported (8)`.
- `rrc_send_err()` is invoked from:
  - `rrc_uart_apply_with_delay()` in `Hiwonder/System/packet_handle.c` (reports UART reconfigure failures under SYS/0xC0).
  - The MOTOR PWM handler in `packet_motor_handle()` for apply failures (SYS/0xEE from FUNC_MOTOR/0x10).
  - The UART baud validator path in `packet_battery_limit_handle()` when an unsupported rate is requested.

## Observations

- Every outbound sub-command emitted via `rrc_send_ack()`, `rrc_send_err()`, or `rrc_transport_send()` is covered by the `g_*_subs` validation tables. IMU preset/bias/primary setters are now implemented inside `packet_imu_handle()` alongside their txid-aware ACK payloads.

## TO FIX

- *(none)*
