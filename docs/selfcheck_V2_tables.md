# Protocol `(func, sub) → max_len` audit (canonical IDs)

Source: `Core/Src/rrclite_packets.c` (`g_*_subs` arrays + `g_func_table`).
All entries now use the canonical `RRC_FUNC_*` identifiers from
`Core/Inc/rrclite_proto.h` / `rrclite_packets.h`.

## RRC_FUNC_SYS (0x00)

| Sub-command | Hex | Max payload (bytes) | Notes |
| --- | --- | --- | --- |
| RRC_SYS_MOTOR_FAILSAFE_SET | 0xB0 | 3 | ACK `{txid, timeout_ms}` |
| RRC_SYS_HEALTH_PERIOD_SET | 0xB1 | 3 | ACK `{txid, period_ms}` |
| RRC_SYS_UART_BAUD_SET | 0xC0 | 7 | ACK `{txid, baud, apply_after_ms}` |
| RRC_SYS_UART_BAUD_GET | 0xC1 | 4 | Response `{baud}` |
| RRC_SYS_PING_ECHO | 0xE0 | 254 | Mirrors request payload |
| RRC_SYS_ERROR_EVENT | 0xEE | 9 | Event `{func, sub, err_code, detail, t_ms, txid}` |
| RRC_SYS_RECOVERED_EVENT | 0xEF | 7 | Event `{func, sub, prev_err, t_ms}` |
| RRC_SYS_VERSION | 0xF0 | 4 | Response `{major, minor, patch}` |
| RRC_SYS_CAPABILITIES | 0xF1 | 16 | Capability bitmap & limits |

## RRC_FUNC_MOTOR (0x10)

| Sub-command | Hex | Max payload (bytes) | Notes |
| --- | --- | --- | --- |
| RRC_MOTOR_PWM_SET | 0x10 | 4 | Request `{motor_id, pwm, [txid]}` |
| RRC_MOTOR_PWM_ACK_SINGLE | 0x03 | 6 | ACK `{txid, motor_id, pwm_target, pwm_applied}` |
| RRC_MOTOR_PWM_ACK_MULTI | 0x19 | 254 | Batch ACK list |

## RRC_FUNC_STEER (0x11)

| Sub-command | Hex | Max payload (bytes) | Notes |
| --- | --- | --- | --- |
| RRC_STEER_SET_POSITION | 0x01 | 255 | Request list of positions (legacy clamp) |
| RRC_STEER_ACK | 0x03 | 2 | ACK `{txid, applied_count}` |

## RRC_FUNC_LED (0x12)

| Sub-command | Hex | Max payload (bytes) | Notes |
| --- | --- | --- | --- |
| RRC_LED_SET | 0x01 | 8 | Request (blink struct + optional txid) |
| RRC_LED_ACK | 0x03 | 2 | ACK `{txid, mode_or_onoff}` |

## RRC_FUNC_BUZZ (0x13)

| Sub-command | Hex | Max payload (bytes) | Notes |
| --- | --- | --- | --- |
| RRC_BUZZ_SET | 0x01 | 9 | Request `{freq, duty, duration, [txid]}` |
| RRC_BUZZ_ACK | 0x03 | 6 | ACK `{txid, freq, duty, duration}` |

## RRC_FUNC_BUTTON (0x20)

| Sub-command | Hex | Max payload (bytes) | Notes |
| --- | --- | --- | --- |
| RRC_BUTTON_STREAM_CTRL | 0x10 | 4 | Request `{enable, period_ms, [txid]}` |
| RRC_BUTTON_ACK | 0x03 | 4 | ACK `{txid, enable, period_ms}` |
| RRC_BUTTON_STREAM_FRAME | 0x11 | 3 | Frame `{seq, mask}` |
| RRC_BUTTON_STREAM_ACK | 0x12 | 2 | Frame-ACK `{seq}` |
| RRC_BUTTON_ONESHOT | 0x20 | 1 | Response `{mask}` |

## RRC_FUNC_ENC (0x21)

| Sub-command | Hex | Max payload (bytes) | Notes |
| --- | --- | --- | --- |
| RRC_ENC_STREAM_CTRL | 0x10 | 4 | Request `{enable, period_ms, [txid]}` |
| RRC_ENC_ACK | 0x03 | 4 | ACK `{txid, enable, period_ms}` |
| RRC_ENC_STREAM_FRAME | 0x11 | 10 | Frame `{seq, c1..c4}` |
| RRC_ENC_STREAM_ACK | 0x12 | 2 | Frame-ACK `{seq}` |
| RRC_ENC_ONESHOT | 0x20 | 10 | Response `{c1..c4}` |

## RRC_FUNC_BATT (0x22)

| Sub-command | Hex | Max payload (bytes) | Notes |
| --- | --- | --- | --- |
| RRC_BATT_STREAM_CTRL | 0x10 | 4 | Request `{enable, period_ms, [txid]}` |
| RRC_BATT_ACK | 0x03 | 4 | ACK `{txid, enable, period_ms}` |
| RRC_BATT_STREAM_FRAME | 0x11 | 4 | Frame `{seq, millivolts}` |
| RRC_BATT_ONESHOT | 0x20 | 2 | Response `{millivolts}` |

## RRC_FUNC_IMU (0x23)

| Sub-command | Hex | Max payload (bytes) | Notes |
| --- | --- | --- | --- |
| RRC_IMU_ONESHOT | 0xA0 | 45 | Request mask + single packed sample |
| RRC_IMU_STREAM_CTRL | 0xA1 | 47 | Largest payload using this sub-ID (stream frame) |
| RRC_IMU_STREAM_ACK | 0xA9 | 3 | Frame-ACK `{source_id, seq}` |
| RRC_IMU_SET_PRIMARY | 0xA2 | 2 | ACK `{txid, source_id}` |
| RRC_IMU_SET_PRESET | 0xA3 | 3 | ACK `{txid, source_id, preset}` |
| RRC_IMU_SET_BIASES | 0xA4 | 2 | ACK `{txid, source_id}` |
| RRC_IMU_WHOAMI_STATUS | 0xA5 | 3 | Response `{source_id, whoami, status}` |

> **Note:** IMU stream control shares sub-ID 0xA1 with the outbound frame for
> backward compatibility; the table therefore records the maximum payload seen
> on that sub-command (the full frame size).

## Findings

* Each `g_*_subs` array is named generically (no legacy "v2" suffixes) and is
  registered under the matching `RRC_FUNC_*` entry in `g_func_table`.
* All expected request/ACK/frame sub-commands have entries with their correct
  maximum payload sizes. No legacy function identifiers remain.

