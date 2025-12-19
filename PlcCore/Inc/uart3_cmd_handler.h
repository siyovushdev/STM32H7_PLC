#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include "main.h"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "semphr.h"


BaseType_t UartTx_Send(const void *msg, size_t len, TickType_t to);        // из тасков
BaseType_t UartTx_SendFromISR(const void *msg, size_t len, BaseType_t *hpw);// из ISR

void u3_start(void);

#ifdef __cplusplus
}
#endif