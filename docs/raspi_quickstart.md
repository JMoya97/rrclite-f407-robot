# RRC Lite – Raspberry Pi Quickstart (Protocol 2)

This guide shows **exact frames** the MCU expects and returns, plus a tiny Python outline for the Pi.

> **Frame format (wire):**  
> `AA 55 | FUNC | LEN | SUB | PAYLOAD... | CRC8-MAXIM`  
> - `FUNC` = function ID (Protocol 2: `SYS=0x00, MOTOR=0x10, STEER=0x11, LED=0x12, BUZZ=0x13, BUTTON=0x20, ENC=0x21, BATT=0x22, IMU=0x23`)  
> - `LEN`  = number of bytes after `SUB` (i.e., payload length)  
> - `CRC`  = CRC-8/MAXIM over `FUNC|LEN|SUB|PAYLOAD...`

## 1) Discover version & capabilities
**Request (Version, SYS/0xF0):**
```
AA 55 | 00 | 00 | F0 | CRC
```
**Response payload:** `{major=2, minor=1, patch=0, reserved=0}`

**Request (Capabilities, SYS/0xF1):**
```
AA 55 | 00 | 00 | F1 | CRC
```
**Response payload:** implementation-defined bitfields (see README).

## 2) Motor PWM set (MOTOR=0x10) + ACK
**Request (SET, SUB=0x01)** — set motor `id`, pwm in `[-1000..1000]`, optional `txid`:
```
AA 55 | 10 | 04 | 01 | id | pwm_lo | pwm_hi | [txid] | CRC
```
**ACK (SUB=0x03)** — echoes `txid`, returns clamped/applied value:
```
AA 55 | 10 | 06 | 03 | txid | id | pwm_lo | pwm_hi | flags | CRC
```

## 3) Encoders stream (ENC=0x21)
**Start stream (CTRL, SUB=0x10)** every `period_ms` (ack echoes txid):
```
AA 55 | 21 | 03 | 10 | period_lo | period_hi | [txid] | CRC
```
**Frame (SUB=0x11)** — `{seq, c1, c2, c3, c4}` all little-endian:
```
AA 55 | 21 | 0A | 11 | seq | c1_lo | c1_hi | c2_lo | c2_hi | c3_lo | c3_hi | c4_lo | c4_hi | CRC
```

## 4) IMU – one-shot & stream (IMU=0x23)
**One-shot (SUB=0x20)** — selects `source_id` (0=ICM-20948 primary):
```
AA 55 | 23 | 01 | 20 | source_id | CRC
```
**Stream control (SUB=0x10)** — `{period_ms, [txid]}`; frames on SUB=0x11 carry `{seq, ax..gz, mx..mz, temp}`.

## 5) UART baud switch (SYS=0x00, SUB=0xC0)
**Request** `{baud_le, apply_after_ms_le, [txid]}`  
ACK at **old** baud; MCU switches after delay; host must change baud within the window.

## 6) Diagnostics (new)
**STATS_GET (SYS/0xF3)** — req=0; resp (18 bytes):  
`{txq_depth,u8, txq_high,u8, drops_imu,u16, drops_enc,u16, drops_batt,u16, drops_btn,u16, err_motor,u16, err_steer,u16, err_imu,u16, err_io,u16}`

**HEALTH_GET (SYS/0xF4)** — req=0; resp=`rrc_health_t` (packed 19 bytes):  
`{system_state,u8, uart_apply_pending,u8, heartbeat_age_ms,u16, failsafe_ms,u16, motor,steer,enc,batt,btn (u8 each), imu0,imu1 (u8 each), motor_err,steer_err,imu_err,io_err (u8 each), retry_in_ms_min,u16}`

**SELFTEST_GET (SYS/0xF5)** — req=0; resp bitmask (u16 LE):  
`{ IMU0(1<<0), IMU1(1<<1), ENCODERS(1<<2), BATTERY(1<<3), UART(1<<4) }`

---

## Python outline (Pi)
> Install: `pip install pyserial`

```python
import serial

POLY = 0x31  # CRC-8/MAXIM (Dallas) reflected form

def crc8_maxim(buf: bytes) -> int:
    crc = 0x00
    for b in buf:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc

def build_frame(func, sub, payload=b""):
    # wire CRC computed over func|len|sub|payload
    body = bytes([func, len(payload), sub]) + payload
    crc = crc8_maxim(body)
    return bytes([0xAA, 0x55]) + body + bytes([crc])

ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.2)
ser.write(build_frame(0x00, 0xF0))          # Version
ser.write(build_frame(0x00, 0xF1))          # Capabilities
ser.write(build_frame(0x21, 0x10, b"\x0A\x00")) # ENC stream @10ms
ser.write(build_frame(0x00, 0xF3))          # STATS_GET
ser.write(build_frame(0x00, 0xF4))          # HEALTH_GET
```

See README “Protocol 2” for more examples and ACK/ERR semantics.

