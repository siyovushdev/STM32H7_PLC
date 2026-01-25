#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifndef PLC_LOG_CAP
#define PLC_LOG_CAP 256
#endif

// События PLC (код + два int16 параметра a/b)
//
// Правило:
// - a/b должны быть маленькими и дешёвыми (int16).
// - если нужно передать uint16/bitmask/crc16 — кладём в (int16_t)(uint16_t)...
// - время ts_ms = now_ms() (uptime ms)
typedef enum {
    PLC_EVT_BOOT = 1,            // a=reset_cause (опц), b=fw_ver (опц)

    // Graph / lifecycle
    PLC_EVT_UPLOAD_OK,           // a=nodeCount, b=cycleMs
    PLC_EVT_UPLOAD_FAIL,         // a=err, b=stage
    PLC_EVT_ACTIVATE_OK,         // a=nodeCount, b=cycleMs
    PLC_EVT_ACTIVATE_FAIL,       // a=err, b=stage

    // Persist (если используешь plc_persist)
    PLC_EVT_PERSIST_SAVE_OK,      // a=bytes_lo16, b=crc16_lo16
    PLC_EVT_PERSIST_SAVE_FAIL,    // a=err, b=stage
    PLC_EVT_PERSIST_LOAD_OK,      // a=bytes_lo16, b=crc16_lo16
    PLC_EVT_PERSIST_LOAD_FAIL,    // a=err, b=stage

    // Protocol / validation
    PLC_EVT_CRC_ERR,             // a=rxCrc_lo16, b=calcCrc_lo16
    PLC_EVT_CBOR_DECODE_ERR,     // a=err, b=offset/ctx
    PLC_EVT_VALIDATE_ERR,        // a=err, b=nodeIndex/ctx

    // Control
    PLC_EVT_FORCE,               // a=nodeIndex, b=value(0/1)
    PLC_EVT_RELEASE,             // a=nodeIndex, b=0

    // Runtime performance
    PLC_EVT_SCAN_OVERRUN,        // a=scanUs (lo16), b=cycleMs (lo16)
    PLC_EVT_SCAN_LONG,           // a=scanUs (lo16), b=cycleMs (lo16)

    // System
    PLC_EVT_WATCHDOG,            // a=0, b=0
    PLC_EVT_ASSERT,              // a=line, b=fileId/ctx

    PLC_EVT_PERSIST_LOAD_REQ,      // a=0, b=0  (load requested)
    // Custom value log (оставил твой старый смысл)
    PLC_EVT_LOG_VALUE            // a=tag, b=value
} PlcEventCode;

typedef struct {
    uint32_t ts_ms;
    uint16_t code;
    int16_t  a;
    int16_t  b;
} PlcLogEntry;

// Ring buffer (как у тебя)
extern PlcLogEntry g_log[PLC_LOG_CAP];
extern uint16_t g_head;
extern uint16_t g_size;

void plc_log_init(void);
void plc_log_put(PlcEventCode code, int16_t a, int16_t b);

// Кол-во текущих записей
uint16_t plc_log_count(void);

// 0..count-1 (старое->новое)
const PlcLogEntry* plc_log_get_by_index(uint16_t idx);

// Для Web/HMI (чтобы gateway мог красиво показывать)
// Не обязательно использовать на STM32, но удобно для теста/CLI
const char* plc_log_event_name(uint16_t code);
