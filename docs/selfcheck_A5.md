# Self-check A5 — protocol documentation refresh

## README updates

- Added **Protocol v2 (behavioral)** section detailing:
  - SYS/0xF0 Version and SYS/0xF1 Capabilities payloads.
  - ACK/NACK flow with optional txid echo, error backoff, and SYS/0xEF recovery notifications.
  - Sequenced streaming with optional per-frame ACK toggle and wrap behaviour.
  - UART baud switching handshake (ACK at old rate, apply-after delay, host re-sync).
  - Motor failsafe timeout behaviour (silent ramp-to-zero while idle).
- Documented compact payload examples for:
  - Motor PWM SET + ACK.
  - Encoder stream control ACK and stream frame.
  - IMU stream frame contents.
  - SYS error/recovered event payloads.

## Status

- ✅ All A5 documentation items captured in README.
