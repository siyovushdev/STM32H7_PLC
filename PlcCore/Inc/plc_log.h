#pragma once
#include <stdint.h>
#include <stdbool.h>

// настроить размер
#define PLC_LOG_CAP 256

typedef enum {
    PLC_EVT_BOOT = 1,
    PLC_EVT_UPLOAD_OK,
    PLC_EVT_UPLOAD_FAIL,
    PLC_EVT_ACTIVATE_OK,
    PLC_EVT_ACTIVATE_FAIL,
    PLC_EVT_CRC_ERR,
    PLC_EVT_VALIDATE_ERR,
    PLC_EVT_FORCE,
    PLC_EVT_STATUS_REQ,
    PLC_EVT_ALERT_LONGSTEP,
    PLC_EVT_LOG_VALUE
} PlcEventCode;

typedef struct {
    uint32_t ts_ms;
    uint16_t code;
    int16_t  a;    // опциональные параметры (маленькие)
    int16_t  b;
} PlcLogEntry;

extern PlcLogEntry g_log[PLC_LOG_CAP];
extern uint16_t g_head;
extern uint16_t g_size;


void plc_log_init(void);
void plc_log_put(PlcEventCode code, int16_t a, int16_t b);
void plc_log_dump_cbor(/*CborEncoder*/ void* root_map); // вызов из send_status_reply() или из отдельной команды
uint16_t plc_log_count(void);
