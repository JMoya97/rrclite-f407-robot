# Protocol v2 Legacy Identifier Audit

This report scans the firmware tree (excluding `Third_Party/`) for protocol v1
symbols that should be retired after the v2 migration. Matches are grouped by
pattern with line references and brief context.

## Summary

| Pattern | Count | Notable files |
| --- | --- | --- |
| BUS_SERVO / 0x06 | 2 | README.md; docs/PROTO_V2_MAP.md |
| IO function 0x04 (LED/BUZZ/BUTTON) | 3 | README.md |
| Encoder subs 0x90/0x91/0x99 | 11 | Core/Inc/rrclite_packets.h; docs/PROTO_V2_MAP.md; README.md; Hiwonder/Misc/checksum.c |
| SYS battery 0xA0/0xA1 | 25 | Core/Inc/rrclite_packets.h; docs/PROTO_V2_MAP.md; docs/selfcheck_A1.md; README.md; Hiwonder/Misc/checksum.c; Core/Src/system_stm32f4xx.c |
| IMU legacy func 0x07 | 27 | Core/Inc/rrclite_packets.h; Hiwonder/System/packet_handle.c; various peripheral headers |

## Detailed findings

### BUS_SERVO (0x06)
- README.md:139 — legacy mapping table still lists “BUS_SERVO (0x06)” → “STEER (0x11)”.
- docs/PROTO_V2_MAP.md:56 — compatibility table documents “BUS_SERVO / `0x01` → STEER / `0x01`”.

### IO function 0x04 (LED/BUZZ/BUTTON)
- Core/Inc/rrclite_packets.h:28 — `#define RRC_FUNC_IO_LEGACY 0x04U` (compat shim constant).
- README.md:135 — legacy entry “IO/LED (0x04, sub 0x20)” in the mapping table.
- README.md:136 — legacy entry “IO/BUZZER (0x04, sub 0x21)”.
- README.md:137 — legacy entry “IO/BUTTON (0x04, subs 0x22/0x23)”.

### Motor encoder subs 0x90/0x91/0x99
- docs/PROTO_V2_MAP.md:50 — notes “MOTOR / `0x90` → ENC / `0x20`”.
- docs/PROTO_V2_MAP.md:51 — notes “MOTOR / `0x91` → ENC / `0x10`”.
- README.md:138 — mapping row cites “MOTOR encoder (0x03, subs 0x90/0x91/0x99)”.
- Core/Inc/rrclite_packets.h:170 — `#define RRC_MOTOR_ENCODER_ONESHOT_LEGACY 0x90U` (compat shim constant).
- Core/Inc/rrclite_packets.h:171 — `#define RRC_MOTOR_ENCODER_STREAM_CTRL_LEGACY 0x91U`.
- Core/Inc/rrclite_packets.h:172 — `#define RRC_MOTOR_ENCODER_STREAM_ACK_LEGACY 0x99U`.
- Hiwonder/Misc/checksum.c:55 — CRC table contains byte value `0x90` (false positive; unrelated to protocol IDs).
- Hiwonder/Misc/checksum.c:55 — same line includes `0x91` (CRC table entry).
- Hiwonder/Misc/checksum.c:58 — CRC table includes `0x99`.

### SYS battery 0xA0 / 0xA1
- docs/PROTO_V2_MAP.md:40 — “SYS / `0xA0` → BATT / `0x20`”.
- docs/PROTO_V2_MAP.md:41 — “SYS / `0xA1` → BATT / `0x10`”.
- docs/selfcheck_A1.md:49 — legacy battery one-shot listed in payload matrix.
- docs/selfcheck_A1.md:50 — same table lists legacy response.
- docs/selfcheck_A1.md:51 — legacy stream-control request row.
- docs/selfcheck_A1.md:52 — legacy stream-control ACK row.
- README.md:134 — mapping row “SYS battery (0xA0/0xA1) → BATT...”.
- Core/Inc/rrclite_packets.h:40 — `RRC_SYS_BATTERY_ONESHOT = 0xA0` (compat shim constant).
- Core/Inc/rrclite_packets.h:41 — `RRC_SYS_BATTERY_STREAM_CTRL = 0xA1`.
- Core/Inc/rrclite_packets.h:285 — `RRC_IMU_ONESHOT = 0xA0` (IMU sub-id shares literal, not battery-specific).
- Core/Inc/rrclite_packets.h:286 — `RRC_IMU_STREAM_CTRL = 0xA1` (IMU literal).
- Hiwonder/Misc/checksum.c:48 — CRC table includes `0xA0`.
- Hiwonder/Misc/checksum.c:49 — CRC table includes `0xA1`.
- Core/Src/system_stm32f4xx.c:478 — register init constant `0xA02A000A` embeds `0xA0` (hardware config; not protocol).
- Core/Src/system_stm32f4xx.c:480 — same constant `0xA02A000A` (hardware config).
- Core/Src/Backup/system_stm32f4xx.c.bak:478 — identical hardware constant `0xA02A000A`.
- Core/Src/Backup/system_stm32f4xx.c.bak:480 — identical hardware constant `0xA02A000A`.

### IMU legacy func 0x07
- Core/Inc/rrclite_packets.h:29 — `#define RRC_FUNC_IMU_LEGACY 0x07U` (compat shim constant).
- Core/Inc/rrclite_packets.h:282 — comment header “IMU (0x07) sub-commands” from legacy block.
- Core/Inc/rrclite_packets.h:285-288 — IMU legacy sub-IDs retain `0xA0/0xA1` etc under the legacy header.
- Hiwonder/System/packet_handle.c:827 — unrelated bus-servo case `0x07` (servo voltage read; legacy peripheral command).
- Hiwonder/System/packet_handle.c:1009 — servo offset case `0x07` (non-IMU command).
- Hiwonder/Misc/checksum.c:37 — CRC table includes `0x07`.
- Core/Src/main.c:72 — loop limit `0x07` (non-protocol constant).
- Core/Src/Backup/main.c.bak:72 — same loop limit.
- Hiwonder/Peripherals/icm20948.c:114 — register constant `0x07`.
- Hiwonder/Peripherals/icm20948.h:22 — register constant `0x07`.
- Hiwonder/Peripherals/QMI8658reg.h:200 — enum `Qmi8658AccOdr_125Hz = 0x06`; nearby values continue around `0x07` (sensor config).
- Hiwonder/Peripherals/QMI8658reg.h:201 — `Qmi8658AccOdr_62_5Hz = 0x07` (sensor config).
- Hiwonder/Peripherals/QMI8658reg.h:233 — `Qmi8658GyrOdr_62_5Hz = 0x07`.
- Hiwonder/Misc/sbus.c:15-31 — SBUS unpacker masks contain `0x07` (bitmask, not a protocol ID).

> **Note:** Many 0x07 matches stem from hardware register values or unrelated legacy peripherals. Only `RRC_FUNC_IMU_LEGACY` and the compatibility table affect packet routing.

