# Protocol Surface Self-Check (Goal A1)

## (FUNC, SUB) payload expectations

| FUNC | SUB | Direction | Max payload (bytes) |
| --- | --- | --- | --- |
| SYS (0x00) | 0xA0 Battery one-shot | req | 0 |
| SYS (0x00) | 0xA0 Battery one-shot | resp | 2 |
| SYS (0x00) | 0xA1 Battery stream ctrl | req | 3 (+1 txid) |
| SYS (0x00) | 0xA1 Battery stream ctrl | resp (ACK) | 4 |
| SYS (0x00) | 0xA1 Battery stream frame | resp (frame) | 4 |
| SYS (0x00) | 0xB0 Motor failsafe set | req | 2 (+1 txid) |
| SYS (0x00) | 0xB0 Motor failsafe set | resp (ACK) | 3 |
| SYS (0x00) | 0xB1 Heartbeat period set | req | 2 (+1 txid) |
| SYS (0x00) | 0xB1 Heartbeat period set | resp (ACK) | 3 |
| SYS (0x00) | 0xC0 UART baud set | req | 6 (+1 txid) |
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
| MOTOR (0x03) | 0x10 PWM set (single) | req | 3 (+1 txid) |
| MOTOR (0x03) | 0x18 PWM ACK (single) | resp | 6 |
| MOTOR (0x03) | 0x19 PWM ACK (multi) | resp | ≤254 |
| MOTOR (0x03) | 0x90 Encoders one-shot | req | 0 |
| MOTOR (0x03) | 0x90 Encoders one-shot | resp | 10 |
| MOTOR (0x03) | 0x91 Encoders stream ctrl | req | 3 (+1 txid) |
| MOTOR (0x03) | 0x91 Encoders stream ctrl | resp (ACK) | 4 |
| MOTOR (0x03) | 0x91 Encoders stream frame | resp (frame) | 10 |
| MOTOR (0x03) | 0x99 Encoder frame ACK | resp | 2 |
| IO (0x04) | 0x20 LEDs set | req | sizeof(LedCommandTypeDef) (+1 txid) or WS2812 length (+1 txid) |
| IO (0x04) | 0x20 LEDs set | resp (ACK) | 2 |
| IO (0x04) | 0x21 Buzzer set | req | 5 (+1 txid) or sizeof(BuzzerCommandTypeDef) (+1 txid) |
| IO (0x04) | 0x21 Buzzer set | resp (ACK) | 6 |
| IO (0x04) | 0x22 Buttons one-shot | req | 0 |
| IO (0x04) | 0x22 Buttons one-shot | resp | 1 |
| IO (0x04) | 0x23 Buttons stream ctrl | req | 3 (+1 txid) |
| IO (0x04) | 0x23 Buttons stream ctrl | resp (ACK) | 4 |
| IO (0x04) | 0x23 Buttons stream frame | resp (frame) | 3 |
| IO (0x04) | 0x29 Buttons frame ACK | resp | 2 |
| IMU (0x07) | 0xA0 IMU one-shot | req | 1 |
| IMU (0x07) | 0xA0 IMU one-shot | resp | 45 |
| IMU (0x07) | 0xA1 IMU stream ctrl | req | 4 (+1 txid) |
| IMU (0x07) | 0xA1 IMU stream ctrl | resp (ACK) | 5 |
| IMU (0x07) | 0xA1 IMU stream frame | resp (frame) | 47 |
| IMU (0x07) | 0xA2 Set primary source | req | 1 (+1 txid) |
| IMU (0x07) | 0xA2 Set primary source | resp (ACK) | 2 |
| IMU (0x07) | 0xA3 Set preset | req | 2 (+1 txid) |
| IMU (0x07) | 0xA3 Set preset | resp (ACK) | 3 |
| IMU (0x07) | 0xA4 Set biases | req | 37 (+1 txid) |
| IMU (0x07) | 0xA4 Set biases | resp (ACK) | 2 |
| IMU (0x07) | 0xA5 WHOAMI/Status | req | 0 (+1 optional source_id) |
| IMU (0x07) | 0xA5 WHOAMI/Status | resp | 3 |
| IMU (0x07) | 0xA9 IMU frame ACK | resp | 3 |

*Request payload lengths are counted after the sub-command byte; values in parentheses denote the optional txid byte or variable legacy formats. “resp” and “event” rows mirror the maxima registered in `g_*_subs` inside `Core/Src/rrclite_packets.c`.*

## Optional txid parsing & ACK echoing

- **Motor PWM set (0x10)** — `packet_motor_handle()` in `Hiwonder/System/packet_handle.c` parses 4- or 5-byte payloads (lines 724–804) and echoes `txid` in `rrc_motor_pwm_ack_t` via `rrc_send_ack()`.
- **Encoder stream ctrl (0x91)** — `packet_encoder_handle()` in `Hiwonder/System/packet_handle.c` (lines 1238–1273) reads the optional `txid`, configures streaming through `encoders_set_stream()`, and replies with `rrc_encoder_stream_ack_t`.
- **Battery stream ctrl (0xA1)** — `packet_battery_limit_handle()` in `Hiwonder/System/packet_handle.c` (lines 1068–1102) supports both 4- and 5-byte payloads and echoes the `txid` in `rrc_sys_battery_stream_ack_t`.
- **Button stream ctrl (0x23)** — `packet_pwm_servo_handle()` in `Hiwonder/System/packet_handle.c` (lines 318–350) handles the optional `txid` and returns `rrc_button_stream_ack_t`.
- **IMU stream ctrl (0xA1)** — `packet_imu_handle()` in `Hiwonder/System/packet_handle.c` (lines 901–937) parses the optional `txid`, calls `imu_set_stream()`, and responds with `rrc_imu_stream_ack_t`.
- **LED set (0x20)** — `packet_led_handle()` in `Hiwonder/System/packet_handle.c` (lines 270–360) recognises legacy and WS2812 payloads, captures the optional `txid`, and emits `rrc_io_led_ack_t`.
- **Buzzer set (0x21)** — `packet_buzzer_handle()` in `Hiwonder/System/packet_handle.c` (lines 380–455) accepts new or legacy payloads, parses the optional `txid`, and echoes it in `rrc_io_buzzer_ack_t`.

## Stream sequence counters

- Encoders — `enc_seq` in `Hiwonder/Portings/encoders_porting.c` (lines 13–110).
- Battery — `batt_seq` in `Hiwonder/System/battery_handle.c` (lines 29–191).
- Buttons — `buttons_seq` in `Hiwonder/Portings/button_porting.c` (lines 13–103).
- IMU primary (ICM-20948) — `imu0_seq` in `Hiwonder/Portings/imu_porting.c` (lines 14–199).
- IMU onboard secondary — no stream counter (source 1 supports one-shot + WHOAMI only); noted for completeness.

## Error codes & emission sites

- `rrc_error_code_t` (`Core/Inc/rrclite_packets.h`, lines 44–51) enumerates: `InvalidArg=1`, `Busy=2`, `Timeout=3`, `IOFail=4`, `NotReady=5`, `NoDevice=6`, `CRCFail=7`, `Unsupported=8`.
- `rrc_send_err()` call sites in `Hiwonder/System/packet_handle.c`:
  - `rrc_uart_apply_with_delay()` (lines 30–47) reports UART reconfigure failures for SYS/0xC0.
  - `packet_led_handle()` (lines 300–361) signals invalid LED arguments (SYS/0xEE from IO/0x20).
  - `packet_buzzer_handle()` (lines 420–455) rejects malformed buzzer payloads.
  - `packet_motor_handle()` (lines 744–780) surfaces PWM apply failures and length errors.
  - `packet_imu_handle()` (lines 948–1040) guards the new 0xA2/0xA3/0xA4 handlers.
  - `packet_battery_limit_handle()` (lines 1172–1180) raises UNSUPPORTED for invalid baud requests before delegating to the UART helper.

## Status & follow-ups

- IMU control handlers for 0xA2/0xA3/0xA4 now live inside `packet_imu_handle()` with local configuration shadows (`g_imu_primary`, `g_imu_preset[]`, `g_imu_bias[]`).
- IO LED (0x20) and Buzzer (0x21) commands emit txid-aware ACKs via `rrc_send_ack()`.
- All active sub-commands dispatched by firmware are represented in the `g_*_subs` max-payload tables; no gaps remain.

## TO FIX

- *(none)*

