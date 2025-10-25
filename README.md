# RRCLite STM32F407 robot

- IDE: STM32CubeIDE 1.19
- MCU: STM32F407VETx + FreeRTOS
- Key files:
  - Hiwonder/Portings/motor_porting.c
  - Hiwonder/System/packet_handle.c
  - Core/Src/tim.c
- Protocol:
  - SYS (0x00): telemetry/control plus ACK/NACK helpers
  - MOTOR (0x03): 0x10 raw PWM with 0x18/0x19 ACK envelopes; encoder stream frames carry `seq`
  - IO (0x04): LEDs, buzzer, and button streaming (optional frame ACK 0x29)
  - IMU (0x07): dual-source stream support with optional frame ACK 0xA9

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
request. The stream ACK sub-commands (0x29, 0x99, 0xA9) simply mirror `{seq}` to
confirm delivery.

Motor PWM command request and ACK payloads (little-endian) are:

```
// MOTOR/0x10 request
uint8_t motor_id;
int16_t pwm;
uint8_t txid; // optional

// MOTOR/0x18 ACK
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
