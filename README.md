# RRCLite STM32F407 robot

- IDE: STM32CubeIDE 1.19
- MCU: STM32F407VETx + FreeRTOS
- Key files:
  - Hiwonder/Portings/motor_porting.c
  - Hiwonder/System/packet_handle.c
  - Core/Src/tim.c
- Protocol:
  - SYS (0x00): telemetry/control plus ACK/NACK helpers
  - MOTOR (0x10): raw PWM set (0x10) with ACK (0x03) and optional batch ACK (0x19); encoder stream frames carry `seq`
  - LED (0x12) / BUZZ (0x13) / BUTTON (0x20): discrete IO controls with txid ACKs and sequenced streams
  - IMU (0x23): dual-source stream support with optional frame ACK 0xA9

### Slim firmware configuration

`Core/Inc/rrclite_config.h` controls which legacy peripherals are compiled into
the slim build. Set `RRC_KEEP_OLED` and `RRC_KEEP_GUI` to `0` (defaults) to skip
creating the historical OLED display and GUI tasks so their stacks are never
allocated. Enabling either flag reinstates the corresponding FreeRTOS thread
and restores the classic behaviour.

## Serial protocol extensions

The current slim + IMU firmware acknowledges every actuator command. Hosts may
attach an optional transaction identifier (txid, `uint8_t`) to supported
requests; legacy payloads without a txid continue to be accepted. When no txid
is supplied, the MCU populates the field with `0xFF` (`RRC_TXID_NONE`) in error
reports so the host can detect the absence. The MCU mirrors the txid in the ACK
payload for positive confirmation. Failures are reported with `SYS/0xEE` while
the firmware attempts a recovery. When the affected module comes back online a
`SYS/0xEF` event is emitted.

Error events use the following payload:

```
uint8_t origin_func;
uint8_t origin_sub;
uint8_t err_code;   // 1=InvalidArg, 2=Busy, 3=Timeout, 4=IOFail,
                    // 5=NotReady, 6=NoDevice, 7=CRCFail, 8=Unsupported
uint8_t detail;     // transport-specific extra info
uint32_t t_ms_le;   // HAL_GetTick() snapshot in milliseconds
uint8_t txid;       // echoed host transaction id when provided
```

When the module recovers the MCU emits:

```
uint8_t origin_func;
uint8_t origin_sub;
uint8_t prev_err_code;
uint32_t t_ms_le;
```

Streaming sensors (battery, encoders, buttons, IMU) prepend a monotonically
increasing `seq` field to each frame. Optional lightweight frame acknowledgers
can be enabled for bring-up via the `ack_each_frame` flag in the stream control
request. The stream ACK sub-commands (0x29, 0x12, 0xA9) simply mirror `{seq}` to
confirm delivery.

Motor PWM command request and ACK payloads (little-endian) are:

```
// MOTOR/0x10 request
uint8_t motor_id;
int16_t pwm;
uint8_t txid; // optional

// MOTOR/0x03 ACK
uint8_t txid;
uint8_t motor_id;
int16_t pwm_target;
int16_t pwm_applied;
```

IMU stream control also carries the txid:

```
// IMU/0xA1 request
uint8_t sources_mask;      // bit0=ICM-20948, bit1=onboard IMU
uint16_t period_ms_le;
uint8_t ack_each_frame;    // 0=default, 1=mirror seq with 0xA9
uint8_t txid;              // optional in legacy mode

// ACK payload (0xA1 response)
uint8_t txid;
uint8_t sources_mask;
uint16_t period_ms_le;
uint8_t ack_each_frame;
```

Encoder stream frames include the sequence counter up front:

```
uint16_t seq;
uint16_t c1;
uint16_t c2;
uint16_t c3;
uint16_t c4;
```

### UART runtime reconfiguration

Hosts can request a baud change using `SYS/0xC0` with `{uint32 baud_le, uint8
txid?}`. Only 115200 and 1,000,000 baud are accepted. The MCU replies at the
current rate with `{txid, baud_le, apply_after_ms_le}` (default 100 ms) and
switches once the delay expires. Hosts must wait for the ACK, reconfigure their
UART within the specified window, and resume communication at the new rate.

### Protocol v2 scaffolding

- `SYS/0xF0` reports the firmware protocol revision `{major=2, minor=0, patch=0}`.
- `SYS/0xF1` exposes capability bits (txid ACKs, sequenced streams, optional
  frame ACKs, dual-IMU streaming, 1 Mbaud support, and motor failsafe) along
  with the maximum supported UART baud (1,000,000 bps) plus representative IMU
  (200 Hz) and encoder (1,000 Hz) streaming rates.

#### Protocol v2 ID map (target)

| Function | ID |
| -------- | -- |
| SYS      | 0x00 |
| MOTOR    | 0x10 |
| STEER    | 0x11 |
| LED      | 0x12 |
| BUZZ     | 0x13 |
| BUTTON   | 0x20 |
| ENC      | 0x21 |
| BATT     | 0x22 |
| IMU      | 0x23 |

| Legacy function (v1) | v2 function |
| -------------------- | ----------- |
| SYS battery (0xA0/0xA1) | BATT (0x22, subs 0x20/0x10/0x11) |
| IO/LED (0x04, sub 0x20) | LED (0x12) |
| IO/BUZZER (0x04, sub 0x21) | BUZZ (0x13) |
| IO/BUTTON (0x04, subs 0x22/0x23) | BUTTON (0x20) |
| MOTOR encoder (0x03, subs 0x90/0x91/0x99) | ENC (0x21, subs 0x20/0x10/0x12) |
| BUS_SERVO (0x06)     | STEER (0x11) |

### Protocol v2 (behavioral)

- **Version & capabilities** — Hosts can query `SYS/0xF0` (Version) for
  `{2,0,0}` and `SYS/0xF1` (Capabilities) for protocol flags, maximum UART baud
  (1,000,000 bps), and representative IMU/encoder rates.
- **ACK/NACK** — Actuator and stream-control requests accept an optional txid
  byte that is echoed in positive ACKs. When hardware rejects a command the
  firmware emits exactly one `SYS/0xEE` error per failure episode, enters an
  exponential backoff, and retries until the device recovers. A successful
  retry generates `SYS/0xEF` (Recovered) and clears the episode without sending
  duplicate ACKs.
- **Sequenced streaming** — Battery, encoder, button, and IMU telemetry frames
  carry a `uint16_t seq` counter that increments per frame and wraps naturally.
  Stream control accepts `ack_each_frame` to enable optional lightweight
  per-frame ACKs; the default remains OFF to conserve bandwidth.
- **Baud switching** — `SYS/0xC0` is acknowledged at the current baud rate.
  Only 115200 and 1,000,000 bps are valid. The ACK advertises the
  apply-after delay; once the host receives it, the MCU waits for the delay and
  then reconfigures the UART. Hosts must re-tune their serial port within that
  window to avoid a link drop.
- **Failsafe** — `SYS/0xB0` programs the motor idle timeout. When no PWM command
  arrives within the configured window the control task ramps targets to zero
  silently (no extra ACKs/errors) and holds until a fresh command is received.

#### Example payloads (little-endian unless noted)

- **Motor PWM set (`MOTOR/0x10`) & ACK (`MOTOR/0x03`)**
  - Request: `{uint8 motor_id, int16 pwm, [uint8 txid]}`
  - ACK: `{uint8 txid, uint8 motor_id, int16 pwm_target, int16 pwm_applied}`
- **Encoder stream control (`ENC/0x10`) & frame**
  - ACK: `{uint8 txid, uint8 enable, uint16 period_ms}`
  - Stream frame (`ENC/0x11` data payload): `{uint16 seq, uint16 c1, uint16 c2,
    uint16 c3, uint16 c4}`
- **IMU stream frame (`IMU/0xA11`)**
  - `{uint8 source_id, uint16 seq, uint32 t_ms, float ax, ay, az, float gx, gy, gz,
     float mx, my, mz, float temp_c}`
- **SYS error (`SYS/0xEE`) & recovered (`SYS/0xEF`) events**
  - Error: `{uint8 origin_func, uint8 origin_sub, uint8 err_code, uint8 detail,
            uint32 t_ms, uint8 txid}`
  - Recovered: `{uint8 origin_func, uint8 origin_sub, uint8 prev_err_code,
                uint32 t_ms}`

## Repository maintenance

To allow local testing of Git operations without an external host, a bare
repository can be created alongside this project and added as the `origin`
remote:

```bash
git init --bare ../rrclite-f407-robot-remote
git remote add origin ../rrclite-f407-robot-remote
git push -u origin work
git remote -v
```

Running `git remote -v` afterwards confirms the repository is connected and
ready for fetch/push operations through the local path. If you want to verify
write access without touching your main branches, create a throwaway branch
and push it to the local remote:

```bash
git switch --create connectivity-check
git push -u origin connectivity-check
git switch -
git branch -D connectivity-check
```
