# Protocol 2 Promotion Audit (P2)

## 1. Legacy identifiers / names
- **FAIL** – Legacy compatibility constants for the v1 function map remain in
  `Core/Inc/rrclite_packets.h` (`RRC_FUNC_MOTOR_LEGACY`, `RRC_FUNC_IO_LEGACY`,
  `RRC_FUNC_IMU_LEGACY`), which still reference the old IDs 0x03/0x04/0x07.
  These must be removed to complete the migration.【F:Core/Inc/rrclite_packets.h†L14-L18】
- No other forbidden tokens were found: `make check-no-legacy` still passes and
  targeted ripgrep probes for shim symbols and `RRCV2_FUNC_` returned no
  matches.【81f5d1†L1-L3】【d55da6†L1-L2】【9f62b1†L1-L2】【35bfd9†L1-L2】

## 2. Canonical enum / constant names
- **PASS** – `rrclite_proto.h` defines `rrc_func_t` with the canonical Protocol 2
  function identifiers (SYS 0x00, MOTOR 0x10, …, IMU 0x23). No `RRCV2_FUNC_*`
  aliases remain.【F:Core/Inc/rrclite_proto.h†L3-L13】

## 3. (func, sub) → max_len tables
- **PASS** – `rrclite_packets.c` registers only the Protocol 2 function groups.
  The table below summarises every entry and its validated payload size.

| Function | Subcommand | Role | Max payload |
| --- | --- | --- | --- |
| SYS (0x00) | FAILSAFE_SET (0xB0) | RX/ACK | `sizeof(rrc_sys_motor_failsafe_ack_t)` |
|  | HEALTH_PERIOD_SET (0xB1) | RX/ACK | `sizeof(rrc_sys_period_ack_t)` |
|  | UART_BAUD_SET (0xC0) | RX/ACK | `sizeof(rrc_sys_uart_baud_ack_t)` |
|  | UART_BAUD_GET (0xC1) | TX | 4 |
|  | PING_ECHO (0xE0) | RX/TX | `RRC_MAX_PAYLOAD_LEN-1` |
|  | ERROR (0xEE) | TX | `sizeof(rrc_sys_error_report_t)` |
|  | RECOVERED (0xEF) | TX | `sizeof(rrc_sys_recovered_report_t)` |
|  | VERSION (0xF0) | TX | `sizeof(rrc_sys_version_resp_t)` |
|  | CAPABILITIES (0xF1) | TX | `sizeof(rrc_sys_capabilities_resp_t)` |
| MOTOR (0x10) | PWM_SET (0x10) | RX | 4 |
|  | PWM_ACK (0x03) | TX | `sizeof(rrc_motor_pwm_ack_t)` |
|  | PWM_ACK_MULTI (0x19) | TX | `RRC_MAX_PAYLOAD_LEN-1` |
| BATT (0x22) | ONE_SHOT (0x20) | TX | 2 |
|  | STREAM_CTRL (0x10) | RX | `sizeof(rrc_batt_stream_ack_t)` |
|  | ACK (0x03) | TX | `sizeof(rrc_batt_stream_ack_t)` |
|  | STREAM_FRAME (0x11) | TX | `sizeof(rrc_batt_stream_frame_t)` |
| ENC (0x21) | ONE_SHOT (0x20) | TX | `sizeof(rrc_encoder_stream_frame_t)` |
|  | STREAM_CTRL (0x10) | RX | `sizeof(rrc_encoder_stream_ack_t)` |
|  | ACK (0x03) | TX | `sizeof(rrc_encoder_stream_ack_t)` |
|  | STREAM_FRAME (0x11) | TX | `sizeof(rrc_encoder_stream_frame_t)` |
|  | STREAM_ACK (0x12) | TX | `sizeof(rrc_encoder_frame_ack_t)` |
| LED (0x12) | SET (0x01) | RX | 8 |
|  | ACK (0x03) | TX | `sizeof(rrc_led_ack_t)` |
| BUZZ (0x13) | SET (0x01) | RX | 9 |
|  | ACK (0x03) | TX | `sizeof(rrc_buzz_ack_t)` |
| BUTTON (0x20) | ONE_SHOT (0x20) | TX | 1 |
|  | STREAM_CTRL (0x10) | RX | `sizeof(rrc_button_stream_ack_t)` |
|  | ACK (0x03) | TX | `sizeof(rrc_button_stream_ack_t)` |
|  | STREAM_FRAME (0x11) | TX | `sizeof(rrc_button_stream_frame_t)` |
|  | STREAM_ACK (0x12) | TX | `sizeof(rrc_button_frame_ack_t)` |
| STEER (0x11) | SET_POSITION (0x01) | RX | `RRC_MAX_PAYLOAD_LEN` |
|  | ACK (0x03) | TX | `sizeof(rrc_steer_ack_t)` |
| IMU (0x23) | ONE_SHOT (0xA0) | TX | `sizeof(rrc_imu_sample_t)` |
|  | STREAM_CTRL (0xA1) | RX/ACK | `sizeof(rrc_imu_stream_frame_t)` |
|  | STREAM_ACK (0xA9) | TX | `sizeof(rrc_imu_frame_ack_t)` |
|  | SET_PRIMARY (0xA2) | RX/ACK | `sizeof(rrc_imu_primary_ack_t)` |
|  | SET_PRESET (0xA3) | RX/ACK | `sizeof(rrc_imu_preset_ack_t)` |
|  | SET_BIASES (0xA4) | RX/ACK | `sizeof(rrc_imu_bias_ack_t)` |
|  | WHOAMI_STATUS (0xA5) | TX | `sizeof(rrc_imu_whoami_resp_t)` |

All rows derive from `g_*_subs` and `g_func_table`, demonstrating that only the
Protocol 2 identifiers are registered.【F:Core/Src/rrclite_packets.c†L32-L108】

## 4. Version and capabilities
- **PASS** – `RRC_PROTO_VERSION_{MAJOR,MINOR,PATCH}` advertise 2.1.0, and the SYS
  handlers return those values alongside the capability bitfield that now only
  encodes Protocol 2-era features (including the new `ids_stable` flag).【F:Core/Inc/rrclite_packets.h†L41-L68】【F:Core/Src/rrclite_packets.c†L144-L170】

## 5. Legacy shims / wire-mode
- **PASS** – No shim or wire-mode files remain, and ripgrep finds no references
  to the removed translation helpers (`rrc_legacy_translate_rx`,
  `rrc_map_outbound`).【aa47c0†L1-L1】【9f62b1†L1-L2】【35bfd9†L1-L2】

## 6. README / docs alignment
- **PASS** – The README consistently refers to “Protocol 2”, lists the canonical
  function IDs (SYS 0x00 … IMU 0x23), and documents sequenced streaming under the
  new function groups.【F:README.md†L9-L159】

### TO FIX
- Remove the remaining `RRC_FUNC_*_LEGACY` compatibility defines (and any other
  references to the v1 function IDs) from `Core/Inc/rrclite_packets.h`.
- Update comments such as “IMU (0x07) sub-commands” to reflect the Protocol 2
  numbering once the legacy defines are gone.【F:Core/Inc/rrclite_packets.h†L270-L281】
