#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Старт TCP сервера (вызывать после MX_LWIP_Init)
bool PlcCommTcp_Start(uint16_t port);

// Останов (опционально)
void PlcCommTcp_Stop(void);

#ifdef __cplusplus
}
#endif
