#include <stddef.h>
#include "plc_log.h"

// Должна быть реализована где-то в plc_hw.c
extern uint32_t now_ms(void);

uint16_t g_size = 0;
uint16_t g_head = 0;

PlcLogEntry g_log[PLC_LOG_CAP];

void plc_log_init(void)
{
    // Старайся вызывать после HAL_Init(), чтобы ts_ms был не 0
    plc_log_put(PLC_EVT_BOOT, 0, 0);
}

void plc_log_put(PlcEventCode code, int16_t a, int16_t b)
{
    PlcLogEntry e;
    e.ts_ms = now_ms();
    e.code  = (uint16_t)code;
    e.a = a;
    e.b = b;

    g_log[g_head] = e;
    g_head = (uint16_t)((g_head + 1u) % PLC_LOG_CAP);
    if (g_size < PLC_LOG_CAP) g_size++;
}

uint16_t plc_log_count(void)
{
    return g_size;
}

const PlcLogEntry* plc_log_get_by_index(uint16_t idx)
{
    if (idx >= g_size) return NULL;

    // start = индекс самого старого элемента
    uint16_t start = (uint16_t)((g_head + PLC_LOG_CAP - g_size) % PLC_LOG_CAP);
    uint16_t real  = (uint16_t)((start + idx) % PLC_LOG_CAP);
    return &g_log[real];
}

const char* plc_log_event_name(uint16_t code)
{
    switch ((PlcEventCode)code) {
        case PLC_EVT_BOOT:               return "BOOT";

        case PLC_EVT_UPLOAD_OK:          return "UPLOAD_OK";
        case PLC_EVT_UPLOAD_FAIL:        return "UPLOAD_FAIL";
        case PLC_EVT_ACTIVATE_OK:        return "ACTIVATE_OK";
        case PLC_EVT_ACTIVATE_FAIL:      return "ACTIVATE_FAIL";

        case PLC_EVT_PERSIST_SAVE_OK:    return "PERSIST_SAVE_OK";
        case PLC_EVT_PERSIST_SAVE_FAIL:  return "PERSIST_SAVE_FAIL";
        case PLC_EVT_PERSIST_LOAD_OK:    return "PERSIST_LOAD_OK";
        case PLC_EVT_PERSIST_LOAD_FAIL:  return "PERSIST_LOAD_FAIL";

        case PLC_EVT_CRC_ERR:            return "CRC_ERR";
        case PLC_EVT_CBOR_DECODE_ERR:    return "CBOR_DECODE_ERR";
        case PLC_EVT_VALIDATE_ERR:       return "VALIDATE_ERR";

        case PLC_EVT_FORCE:              return "FORCE";
        case PLC_EVT_RELEASE:            return "RELEASE";

        case PLC_EVT_SCAN_OVERRUN:       return "SCAN_OVERRUN";
        case PLC_EVT_SCAN_LONG:          return "SCAN_LONG";

        case PLC_EVT_WATCHDOG:           return "WATCHDOG";
        case PLC_EVT_ASSERT:             return "ASSERT";

        case PLC_EVT_LOG_VALUE:          return "LOG_VALUE";

        default:                         return "UNKNOWN";
    }
}
