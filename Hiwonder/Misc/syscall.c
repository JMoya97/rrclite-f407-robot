/// 标准库桩函数的重定向


#include <stdio.h>
#include "usart.h"


int stdin_getchar (void)
{
    return 0;
}
int stderr_putchar (int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
    // SEGGER_RTT_Write(0, &ch, 1);
    return ch;
}
int stdout_putchar (int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
    // SEGGER_RTT_Write(0, &ch, 1);
    return (ch);
}

void ttywrch (int ch) {
}
