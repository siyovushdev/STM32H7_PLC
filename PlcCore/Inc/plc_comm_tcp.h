#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#ifndef PLC_TCP_MAX_FRAMES_PER_PUMP
#define PLC_TCP_MAX_FRAMES_PER_PUMP 4u
#endif

// Старт TCP сервера (вызывать после MX_LWIP_Init)
bool PlcCommTcp_Start(uint16_t port);

// Останов (опционально)
void PlcCommTcp_Stop(void);

#ifdef __cplusplus
}
#endif
