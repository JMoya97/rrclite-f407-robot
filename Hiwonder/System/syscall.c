/// 标准库桩函数的重定向


#include <stdio.h>
#include "usart.h"
#include "global.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE* f)
#endif /* __GNUC__ */
 
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus
 

PUTCHAR_PROTOTYPE
{
  //HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF); /* 重定向到 UART1 打印 */
	SEGGER_RTT_Write(0, &ch, 1);   /* 重定向到 JLINK RTT 打印 */
  return (ch);
}


#ifdef __cplusplus
}
#endif //__cplusplus

