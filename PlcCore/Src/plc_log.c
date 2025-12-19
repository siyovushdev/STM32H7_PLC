#include <stddef.h>
#include "plc_log.h"

extern uint32_t now_ms(void);

uint16_t g_size = 0;
uint16_t g_head = 0;

PlcLogEntry g_log[PLC_LOG_CAP];

void plc_log_init(void){
    plc_log_put(PLC_EVT_BOOT, 0, 0);
}

void plc_log_put(PlcEventCode code, int16_t a, int16_t b){
    PlcLogEntry e;
    e.ts_ms = now_ms();
    e.code  = (uint16_t)code;
    e.a = a; e.b = b;
    g_log[g_head] = e;
    g_head = (g_head + 1) % PLC_LOG_CAP;
    if (g_size < PLC_LOG_CAP) g_size++;
}

uint16_t plc_log_count(void){ return g_size; }

const PlcLogEntry* plc_log_get_by_index(uint16_t idx)
{
    if (idx >= g_size) return NULL;
    uint16_t start = (g_head + PLC_LOG_CAP - g_size) % PLC_LOG_CAP;
    uint16_t real = (start + idx) % PLC_LOG_CAP;
    return &g_log[real];
}

