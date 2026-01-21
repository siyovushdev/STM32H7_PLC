#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "plc.h"          // PlcGraph, PlcNode, PLC_MAX_NODES, PLC_MEM_*_COUNT

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CBOR_MAX_REPLY
#define CBOR_MAX_REPLY 8192u
#endif

typedef struct {
    uint8_t data[CBOR_MAX_REPLY];
    size_t  len;     // 0, если ответа нет
} CborReply;

// ======================= CMD/REPLY (transport <-> PLC task) =======================

typedef enum {
    PLC_CMD_NONE = 0,
    PLC_CMD_UPLOAD_GRAPH,
    PLC_CMD_ACTIVATE_GRAPH,
    PLC_CMD_STATUS,
    PLC_CMD_SAVE_ACTIVE,
    PLC_CMD_LOAD_ACTIVE,
    PLC_CMD_LOG_DUMP,
    PLC_CMD_FORCE_OUTPUT,
    PLC_CMD_RELEASE_OUTPUT,
    PLC_CMD_MEM_READ,
    PLC_CMD_MEM_WRITE,
    PLC_CMD_MEM_RESET,
    PLC_CMD_MEM_INFO,
} PlcCmdType;

typedef enum {
    PLC_MEM_T_BOOL = 1,
    PLC_MEM_T_INT  = 2,
    PLC_MEM_T_REAL = 3,
} PlcMemType;

typedef struct {
    PlcCmdType type;
    union {
        struct { PlcGraph graph; } upload;

        struct { uint16_t nodeIndex; bool value; uint32_t holdMs; } force;
        struct { uint16_t nodeIndex; } release;

        struct { PlcMemType memType; uint16_t from; uint16_t count; } memRead;

        struct {
            PlcMemType memType;
            uint16_t from;
            uint16_t count;
            union {
                bool    b[PLC_MEM_BOOL_COUNT];
                int32_t i[PLC_MEM_INT_COUNT];
                float   f[PLC_MEM_REAL_COUNT];
            } v;
        } memWrite;
    } u;
} PlcCmd;

// ----- status snapshot -----
typedef struct {
    uint16_t id;
    uint16_t type;
    uint8_t  valueType;

    union { bool b; int32_t i; float f; } out;

    // debug/extra
    uint32_t tonMs;
    uint32_t toffLeftMs;

    // PID debug
    float sp, pv, i_term, u;

    // force
    bool     forceActive;
    bool     forceValue;
    uint32_t forceLeftMs;
} PlcNodeSnapshot;

typedef struct {
    uint32_t uptimeMs;
    bool     active;
    uint32_t cycleMs;
    uint16_t nodesCount;

    // metrics
    uint32_t scanTickAvgUs, scanTickMaxUs;
    uint32_t scanWorkAvgUs, scanWorkMaxUs;
    uint32_t cycleRealAvgMs, cycleRealMaxMs;
    uint32_t scanLongSteps;

    PlcNodeSnapshot nodes[PLC_MAX_NODES];
} PlcStatusSnapshot;

// ----- log dump snapshot (подстрой под твой plc_log API) -----
#ifndef PLC_LOG_DUMP_MAX
#define PLC_LOG_DUMP_MAX 64u
#endif

typedef struct {
    uint32_t tsMs;
    int16_t  code;
    int16_t  a;
    int16_t  b;
} PlcLogItemSnapshot;

typedef struct {
    uint16_t count;
    PlcLogItemSnapshot items[PLC_LOG_DUMP_MAX];
} PlcLogDumpSnapshot;

// ----- mem replies -----
typedef struct {
    PlcMemType memType;
    uint16_t from;
    uint16_t count;
    union {
        bool    b[PLC_MEM_BOOL_COUNT];
        int32_t i[PLC_MEM_INT_COUNT];
        float   f[PLC_MEM_REAL_COUNT];
    } v;
} PlcMemReadSnapshot;

typedef struct {
    uint16_t boolCount;
    uint16_t intCount;
    uint16_t realCount;
} PlcMemInfoSnapshot;

// ----- replies -----
typedef enum {
    PLC_REPLY_NONE = 0,
    PLC_REPLY_UPLOAD_GRAPH_ACK,
    PLC_REPLY_ACTIVATE_GRAPH_ACK,
    PLC_REPLY_STATUS,
    PLC_REPLY_SAVE_ACTIVE_ACK,
    PLC_REPLY_LOAD_ACTIVE_ACK,
    PLC_REPLY_LOG_DUMP,
    PLC_REPLY_FORCE_OUTPUT_ACK,
    PLC_REPLY_RELEASE_OUTPUT_ACK,
    PLC_REPLY_MEM_READ,
    PLC_REPLY_MEM_WRITE_ACK,
    PLC_REPLY_MEM_RESET_ACK,
    PLC_REPLY_MEM_INFO,
    PLC_REPLY_ERROR, // unknown cmd / parse error / etc.
} PlcReplyType;

typedef struct {
    PlcReplyType type;
    bool ok;
    uint32_t aux; // written count / etc (по месту)

    union {
        PlcStatusSnapshot   status;
        PlcLogDumpSnapshot  logDump;
        PlcMemReadSnapshot  memRead;
        PlcMemInfoSnapshot  memInfo;
    } u;
} PlcReply;

// ======================= Broker (IO -> PLC) =======================

bool PlcMsg_Init(uint32_t depth);

/**
 * IO thread: RPC по структурам.
 * cmd уже ДЕКОДИРОВАН (CBOR decode в IO).
 * PLC task применит команду и заполнит reply.
 */
bool PlcMsg_RpcCallCmd(const PlcCmd* cmd, PlcReply* reply, uint32_t timeoutMs);

/**
 * PLC task ONLY: обработать команды в пределах budgetUs.
 * timeoutMsFirst: ждать первую команду (0 = не ждать).
 * maxItems: максимум команд за вызов.
 * Возвращает сколько обработал.
 */
uint32_t PlcMsg_PumpApply(uint32_t maxItems, uint32_t timeoutMsFirst, uint32_t budgetUs);

// ======================= CBOR (IO side) =======================

/** IO side: decode CBOR payload -> PlcCmd */
bool CborProto_DecodeCmd(const uint8_t* buf, size_t len, PlcCmd* outCmd);

/** IO side: encode PlcReply -> CBOR payload */
bool CborProto_EncodeReply(const PlcReply* reply, CborReply* outBytes);

// counters (по желанию)
void proto_count_rx_frame_ok(void);
void proto_count_rx_crc_err(void);
void proto_count_rx_parse_err(void);
void proto_count_tx_frame(void);

#ifdef __cplusplus
}
#endif
