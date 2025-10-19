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
