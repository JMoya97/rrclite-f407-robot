# Protocol v2 Mapping (Scaffolding)

## Version & Compatibility Flags

- `RRC_PROTO_VERSION_MAJOR = 2`
- `RRC_PROTO_VERSION_MINOR = 0`
- `RRC_PROTO_COMPAT_LEGACY = 1` (compile-time gate for translating legacy v1 packets)

## Function ID Allocation

| v2 Function | Value | Legacy coverage |
|-------------|-------|-----------------|
| `RRC_FUNC_SYS`    | `0x00` | System commands, heartbeat, version, capabilities |
| `RRC_FUNC_MOTOR`  | `0x10` | Motor PWM set / telemetry |
| `RRC_FUNC_STEER`  | `0x11` | Bus-servo steering commands |
| `RRC_FUNC_LED`    | `0x12` | LED output commands |
| `RRC_FUNC_BUZZ`   | `0x13` | Buzzer output commands |
| `RRC_FUNC_BUTTON` | `0x20` | Button one-shot & stream control |
| `RRC_FUNC_ENC`    | `0x21` | Encoder one-shot & stream control |
| `RRC_FUNC_BATT`   | `0x22` | Battery one-shot & stream control |
| `RRC_FUNC_IMU`    | `0x23` | IMU data & configuration |

Common sub-ID conventions:

- `0x01` → `SET`
- `0x02` → `GET`
- `0x03` → `ACK`
- `0x10` → `STREAM_CTRL`
- `0x11` → `STREAM_FRAME`
- `0x12` → `STREAM_ACK`
- `0x20` → `ONE_SHOT`
- IMU configuration reserves `0x31` (preset) and `0x32` (biases).

## Legacy (v1) → v2 Translation Table

`rrc_legacy_translate_rx()` rewrites host-originated v1 packets before validation when `RRC_PROTO_COMPAT_LEGACY == 1`.

| Legacy func/sub | v2 func/sub | Notes |
|-----------------|-------------|-------|
| SYS / `0xA0` | BATT / `0x20` (`ONE_SHOT`) | Battery millivolt query |
| SYS / `0xA1` | BATT / `0x10` (`STREAM_CTRL`) | Battery stream control |
| SYS / `0xB0` | SYS / `0xB0` | Motor failsafe timeout |
| SYS / `0xB1` | SYS / `0xB1` | Heartbeat period |
| SYS / `0xC0` | SYS / `0xC0` | UART baud set |
| SYS / `0xC1` | SYS / `0xC1` | UART baud get |
| SYS / `0xE0` | SYS / `0xE0` | Echo |
| SYS / `0xF0` | SYS / `0xF0` | Version query |
| SYS / `0xF1` | SYS / `0xF1` | Capabilities query |
| MOTOR / `0x10` | MOTOR / `0x01` (`SET`) | PWM set |
| MOTOR / `0x90` | ENC / `0x20` (`ONE_SHOT`) | Encoder one-shot |
| MOTOR / `0x91` | ENC / `0x10` (`STREAM_CTRL`) | Encoder stream control |
| IO / `0x20` | LED / `0x01` (`SET`) | LED mode / WS2812 frame |
| IO / `0x21` | BUZZ / `0x01` (`SET`) | Buzzer frequency/duty |
| IO / `0x22` | BUTTON / `0x20` (`ONE_SHOT`) | Button state query |
| IO / `0x23` | BUTTON / `0x10` (`STREAM_CTRL`) | Button stream control |
| BUS_SERVO / `0x01` | STEER / `0x01` (`SET`) | Bus-servo target apply |
| IMU / `0xA0` | IMU / `0x20` (`ONE_SHOT`) | IMU sample request |
| IMU / `0xA1` | IMU / `0x10` (`STREAM_CTRL`) | IMU stream control |
| IMU / `0xA2` | IMU / `0x01` (`SET`) | Primary source select |
| IMU / `0xA3` | IMU / `0x31` | ODR/LPF preset |
| IMU / `0xA4` | IMU / `0x32` | Bias calibration |
| IMU / `0xA5` | IMU / `0x02` (`GET`) | WHOAMI / status |

This scaffold does not modify runtime behaviour; the shim is currently defined but not yet invoked.
