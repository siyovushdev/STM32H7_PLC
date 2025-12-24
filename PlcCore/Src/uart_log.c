#include "uart_log.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

void UartLog_Init(void)
{
    // UART уже инициализирован CubeMX-ом (MX_USART3_UART_Init)
}

void UartLog_Printf(const char* fmt, ...)
{
    char buf[256];

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0) return;
    if (n > (int)sizeof(buf)) n = (int)sizeof(buf);

    // Блокирующая передача (для диагностики ок)
    HAL_UART_Transmit(&huart3, (uint8_t*)buf, (uint16_t)n, 100);
}
