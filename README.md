# RRCLite STM32F407 robot

- IDE: STM32CubeIDE 1.19
- MCU: STM32F407VETx + FreeRTOS
- Key files:
  - Hiwonder/Portings/motor_porting.c
  - Hiwonder/System/packet_handle.c
  - Core/Src/tim.c
- Protocol:
  - MOTOR (0x03): 0x10 raw PWM; **planned** 0x18/0x19 ACK
  - ENC (0x03): 0x90 one-shot; 0x91 stream
  - IMU (0x07): A0 one-shot; A1 stream

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
