# Protocol v2 `(func, sub) → max_len` inventory

Source: `Core/Src/rrclite_packets.c` (lookup tables `g_*_subs` + `g_func_table`). All
entries below reference the protocol v2 function identifiers defined in
`Core/Inc/rrclite_packets.h` / `rrclite_proto.h`.

## RRCV2_FUNC_SYS (0x00)

| Sub-command | Hex | Direction | Max payload (bytes) | Notes |
| --- | --- | --- | --- | --- |
| RRC_SYS_MOTOR_FAILSAFE_SET | 0xB0 | TX (ACK) | 3 | `{txid, timeout_ms}`
| RRC_SYS_HEALTH_PERIOD_SET | 0xB1 | TX (ACK) | 3 | `{txid, period_ms}`
| RRC_SYS_UART_BAUD_SET | 0xC0 | TX (ACK) | 7 | `{txid, baud, apply_after_ms}`
| RRC_SYS_UART_BAUD_GET | 0xC1 | TX (RESP) | 4 | `{baud}`
| RRC_SYS_PING_ECHO | 0xE0 | TX (RESP) | 254 | Mirrors request payload
| RRC_SYS_ERROR_EVENT | 0xEE | TX (EVENT) | 9 | `{func, sub, err_code, detail, t_ms, txid}`
| RRC_SYS_RECOVERED_EVENT | 0xEF | TX (EVENT) | 7 | `{func, sub, prev_err, t_ms}`
| RRC_SYS_VERSION | 0xF0 | TX (RESP) | 4 | `{major, minor, patch}`
| RRC_SYS_CAPABILITIES | 0xF1 | TX (RESP) | 16 | Capability bitmap & limits

## RRCV2_FUNC_MOTOR (0x10)

| Sub-command | Hex | Direction | Max payload (bytes) | Notes |
| --- | --- | --- | --- | --- |
| RRC_MOTOR_PWM_ACK_SINGLE | 0x03 | TX (ACK) | 6 | `{txid, motor_id, pwm_target, pwm_applied}`
| RRC_MOTOR_PWM_ACK_MULTI | 0x19 | TX (RESP) | 254 | Batch ACK list

## RRCV2_FUNC_STEER (0x11)

| Sub-command | Hex | Direction | Max payload (bytes) | Notes |
| --- | --- | --- | --- | --- |
| RRC_STEER_SET_POSITION | 0x01 | RX (REQ) | 255 | Parsed + validated in handler
| RRC_STEER_ACK | 0x03 | TX (ACK) | 2 | `{txid, applied_count}`

## RRCV2_FUNC_LED (0x12)

| Sub-command | Hex | Direction | Max payload (bytes) | Notes |
| --- | --- | --- | --- | --- |
| RRC_LED_ACK | 0x03 | TX (ACK) | 2 | `{txid, mode_or_onoff}`

## RRCV2_FUNC_BUZZ (0x13)

| Sub-command | Hex | Direction | Max payload (bytes) | Notes |
| --- | --- | --- | --- | --- |
| RRC_BUZZ_ACK | 0x03 | TX (ACK) | 6 | `{txid, freq_hz, duty_pct, duration_ms}`

## RRCV2_FUNC_BUTTON (0x20)

| Sub-command | Hex | Direction | Max payload (bytes) | Notes |
| --- | --- | --- | --- | --- |
| RRC_BUTTON_STREAM_CTRL | 0x10 | RX (REQ) | 4 | `{enable, period_ms, [txid]}`
| RRC_BUTTON_ACK | 0x03 | TX (ACK) | 4 | `{txid, enable, period_ms}`
| RRC_BUTTON_STREAM_FRAME | 0x11 | TX (FRAME) | 3 | `{seq, mask}`
| RRC_BUTTON_STREAM_ACK | 0x12 | TX (FRAME-ACK) | 2 | `{seq}`
| RRC_BUTTON_ONESHOT | 0x20 | TX (RESP) | 1 | `{mask}`

## RRCV2_FUNC_ENC (0x21)

| Sub-command | Hex | Direction | Max payload (bytes) | Notes |
| --- | --- | --- | --- | --- |
| RRC_ENC_STREAM_CTRL | 0x10 | RX (REQ) | 4 | `{enable, period_ms, [txid]}`
| RRC_ENC_ACK | 0x03 | TX (ACK) | 4 | `{txid, enable, period_ms}`
| RRC_ENC_STREAM_FRAME | 0x11 | TX (FRAME) | 10 | `{seq, c1..c4}`
| RRC_ENC_STREAM_ACK | 0x12 | TX (FRAME-ACK) | 2 | `{seq}`
| RRC_ENC_ONESHOT | 0x20 | TX (RESP) | 10 | `{c1..c4}`

## RRCV2_FUNC_BATT (0x22)

| Sub-command | Hex | Direction | Max payload (bytes) | Notes |
| --- | --- | --- | --- | --- |
| RRC_BATT_STREAM_CTRL | 0x10 | RX (REQ) | 4 | `{enable, period_ms, [txid]}`
| RRC_BATT_ACK | 0x03 | TX (ACK) | 4 | `{txid, enable, period_ms}`
| RRC_BATT_STREAM_FRAME | 0x11 | TX (FRAME) | 4 | `{seq, millivolts}`
| RRC_BATT_ONESHOT | 0x20 | TX (RESP) | 2 | `{millivolts}`

## RRCV2_FUNC_IMU (0x23)

| Sub-command | Hex | Direction | Max payload (bytes) | Notes |
| --- | --- | --- | --- | --- |
| RRC_IMU_ONESHOT | 0xA0 | RX/RESP | 45 | Request mask; response per-source packed sample
| RRC_IMU_STREAM_CTRL | 0xA1 | RX/ACK | 47 | `{sources_mask, period_ms, ack_each_frame, [txid]}`
| RRC_IMU_STREAM_ACK | 0xA9 | TX (FRAME-ACK) | 3 | `{source_id, seq}`
| RRC_IMU_SET_PRIMARY | 0xA2 | RX/ACK | 2 | `{txid, source_id}`
| RRC_IMU_SET_PRESET | 0xA3 | RX/ACK | 3 | `{txid, source_id, preset}`
| RRC_IMU_SET_BIASES | 0xA4 | RX/ACK | 2 | `{txid, source_id}`
| RRC_IMU_WHOAMI_STATUS | 0xA5 | TX (RESP) | 3 | `{source_id, whoami, status}`

> **Note:** IMU retains the 0xA* sub-command values for backward compatibility.

## Findings

* Every entry in `g_func_table` now references a v2 function identifier; no legacy
  (v1) function IDs remain in the dispatch table.
* Expected sub-commands for each function are present with the correct maximum
  payload sizes. No missing rows were observed, so no “TO FIX” items are
  required.

