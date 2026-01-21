#include "cbor_msg.h"
#include "plc_persist.h"
#include "logf.h"
#include "cbor.h"
#include "plc.h"
#include "plc_log.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

static const char *tag = "CBOR";

// ====== PLC BROKER (IO -> PLC task), NO HEAP ======

typedef struct {
    osEventFlagsId_t ev;   // pre-created
    volatile uint8_t inUse;
    PlcCmd   cmd;
    PlcReply reply;
} PlcRpcSlot;

typedef struct {
    uint16_t slotIndex;
} PlcQItem;

static osMessageQueueId_t s_plcQ = NULL;

#ifndef PLC_RPC_SLOTS_MAX
#define PLC_RPC_SLOTS_MAX 8u
#endif

static PlcRpcSlot s_slots[PLC_RPC_SLOTS_MAX];

static int alloc_slot(void)
{
    taskENTER_CRITICAL();
    for (uint32_t i = 0; i < PLC_RPC_SLOTS_MAX; i++) {
        if (!s_slots[i].inUse) {
            s_slots[i].inUse = 1u;
            taskEXIT_CRITICAL();
            return (int)i;
        }
    }
    taskEXIT_CRITICAL();
    return -1;
}

static void free_slot(uint32_t i)
{
    taskENTER_CRITICAL();
    s_slots[i].inUse = 0u;
    taskEXIT_CRITICAL();
}

bool PlcMsg_Init(uint32_t depth)
{
    if (s_plcQ) return true;

    if (depth == 0u) depth = 8u;
    if (depth > PLC_RPC_SLOTS_MAX) depth = PLC_RPC_SLOTS_MAX;

    // init slots
    for (uint32_t i = 0; i < PLC_RPC_SLOTS_MAX; i++) {
        s_slots[i].inUse = 0u;
        if (!s_slots[i].ev) {
            s_slots[i].ev = osEventFlagsNew(NULL);
            if (!s_slots[i].ev) return false;
        }
    }

    s_plcQ = osMessageQueueNew(depth, sizeof(PlcQItem), NULL);
    return (s_plcQ != NULL);
}

bool PlcMsg_RpcCallCmd(const PlcCmd* cmd, PlcReply* reply, uint32_t timeoutMs)
{
    if (!s_plcQ || !cmd || !reply) return false;

    int si = alloc_slot();
    if (si < 0) return false;

    // copy cmd to slot
    PlcRpcSlot* s = &s_slots[si];
    memcpy(&s->cmd, cmd, sizeof(PlcCmd));
    memset(&s->reply, 0, sizeof(PlcReply));

    // reset event
    (void)osEventFlagsClear(s->ev, 0x1u);

    PlcQItem qi = { .slotIndex = (uint16_t)si };
    if (osMessageQueuePut(s_plcQ, &qi, 0, 0) != osOK) {
        free_slot((uint32_t)si);
        return false;
    }

    uint32_t flags = osEventFlagsWait(s->ev, 0x1u, osFlagsWaitAny, timeoutMs);
    if ((int32_t)flags < 0) {
        // PLC может обработать позже, но слот нам нужен. Считаем таймаутом.
        free_slot((uint32_t)si);
        return false;
    }

    memcpy(reply, &s->reply, sizeof(PlcReply));
    free_slot((uint32_t)si);
    return true;
}

// ====== PLC APPLY (single-writer, PLC task only) ======

static void fill_status_snapshot(PlcStatusSnapshot* s)
{
    memset(s, 0, sizeof(*s));
    s->uptimeMs = now_ms();

    bool active = g_activeGraphValid;
    s->active = active;
    s->cycleMs = active ? g_activeGraph.cycleMs : 0;
    s->nodesCount = active ? g_activeGraph.nodeCount : 0;

    s->scanTickAvgUs = g_scan_cnt ? (uint32_t)(g_scan_us_accum / g_scan_cnt) : 0;
    s->scanTickMaxUs = g_scan_us_max;

    s->scanWorkAvgUs = g_work_cnt ? (uint32_t)(g_work_us_accum / g_work_cnt) : 0;
    s->scanWorkMaxUs = g_work_us_max;

    s->cycleRealAvgMs = g_cycle_cnt ? (uint32_t)(g_cycle_ms_accum / g_cycle_cnt) : 0;
    s->cycleRealMaxMs = g_cycle_ms_max;

    s->scanLongSteps = g_scan_long_steps;

    if (active) {
        for (uint16_t i = 0; i < s->nodesCount && i < PLC_MAX_NODES; i++) {
            PlcNode* n = &g_activeGraph.nodes[i];
            PlcNodeSnapshot* o = &s->nodes[i];

            o->id = n->id;
            o->type = n->type;
            o->valueType = (uint8_t)n->valueType;

            if (n->valueType == PLC_VAL_BOOL) o->out.b = n->out.b;
            else if (n->valueType == PLC_VAL_INT) o->out.i = n->out.i;
            else o->out.f = n->out.f;

            if (n->type == PLC_NODE_TON) o->tonMs = n->ton_accum_ms;
            if (n->type == PLC_NODE_TOFF) o->toffLeftMs = n->toff_left_ms;

            if (n->type == PLC_NODE_PID) {
                float sp = 0.0f, pv = 0.0f;

                int16_t ia = n->inA;
                int16_t ib = n->inB;

                if (ia >= 0 && (uint16_t)ia < s->nodesCount) sp = g_activeGraph.nodes[ia].out.f;
                if (ib >= 0 && (uint16_t)ib < s->nodesCount) pv = g_activeGraph.nodes[ib].out.f;

                o->sp = sp;
                o->pv = pv;
                o->i_term = n->pid_i;
                o->u = n->out.f;
            }


            if (n->flags & PLC_NODE_FLAG_FORCE) {
                o->forceActive = true;
                o->forceValue = n->force_bool;
                o->forceLeftMs = n->force_left_ms;
            }
        }
    }
}

static void fill_mem_info(PlcMemInfoSnapshot* m)
{
    m->boolCount = PLC_MEM_BOOL_COUNT;
    m->intCount  = PLC_MEM_INT_COUNT;
    m->realCount = PLC_MEM_REAL_COUNT;
}

static void fill_mem_read(const PlcCmd* cmd, PlcMemReadSnapshot* out)
{
    memset(out, 0, sizeof(*out));
    out->memType = cmd->u.memRead.memType;
    out->from = cmd->u.memRead.from;
    out->count = cmd->u.memRead.count;

    uint16_t n = out->count;
    if (out->memType == PLC_MEM_T_BOOL) {
        if (out->from + n > PLC_MEM_BOOL_COUNT) n = (out->from < PLC_MEM_BOOL_COUNT) ? (PLC_MEM_BOOL_COUNT - out->from) : 0;
        out->count = n;
        for (uint16_t i = 0; i < n; i++) out->v.b[i] = plc_mem_get_bool((uint16_t)(out->from + i));
    } else if (out->memType == PLC_MEM_T_INT) {
        if (out->from + n > PLC_MEM_INT_COUNT) n = (out->from < PLC_MEM_INT_COUNT) ? (PLC_MEM_INT_COUNT - out->from) : 0;
        out->count = n;
        for (uint16_t i = 0; i < n; i++) out->v.i[i] = plc_mem_get_int((uint16_t)(out->from + i));
    } else {
        if (out->from + n > PLC_MEM_REAL_COUNT) n = (out->from < PLC_MEM_REAL_COUNT) ? (PLC_MEM_REAL_COUNT - out->from) : 0;
        out->count = n;
        for (uint16_t i = 0; i < n; i++) out->v.f[i] = plc_mem_get_real((uint16_t)(out->from + i));
    }
}

static void apply_mem_write(const PlcCmd* cmd, PlcReply* rep)
{
    uint16_t from = cmd->u.memWrite.from;
    uint16_t n    = cmd->u.memWrite.count;

    if (cmd->u.memWrite.memType == PLC_MEM_T_BOOL) {
        if (from + n > PLC_MEM_BOOL_COUNT) n = (from < PLC_MEM_BOOL_COUNT) ? (PLC_MEM_BOOL_COUNT - from) : 0;
        for (uint16_t i = 0; i < n; i++) plc_mem_set_bool((uint16_t)(from + i), cmd->u.memWrite.v.b[i]);
    } else if (cmd->u.memWrite.memType == PLC_MEM_T_INT) {
        if (from + n > PLC_MEM_INT_COUNT) n = (from < PLC_MEM_INT_COUNT) ? (PLC_MEM_INT_COUNT - from) : 0;
        for (uint16_t i = 0; i < n; i++) plc_mem_set_int((uint16_t)(from + i), cmd->u.memWrite.v.i[i]);
    } else {
        if (from + n > PLC_MEM_REAL_COUNT) n = (from < PLC_MEM_REAL_COUNT) ? (PLC_MEM_REAL_COUNT - from) : 0;
        for (uint16_t i = 0; i < n; i++) plc_mem_set_real((uint16_t)(from + i), cmd->u.memWrite.v.f[i]);
    }

    rep->ok = true;
    rep->aux = n; // written
}


static void fill_log_dump(PlcLogDumpSnapshot* out)
{
    memset(out, 0, sizeof(*out));

    uint16_t n = plc_log_count();
    if (n == 0) { out->count = 0; return; }

    // берём последние PLC_LOG_DUMP_MAX записей
    uint16_t take = (n > PLC_LOG_DUMP_MAX) ? PLC_LOG_DUMP_MAX : n;
    out->count = take;

    // индекс в логе: 0..n-1 (старое->новое)
    // надо взять последние: [n-take .. n-1]
    uint16_t start = (uint16_t)(n - take);

    for (uint16_t i = 0; i < take; i++) {
        const PlcLogEntry* e = plc_log_get_by_index((uint16_t)(start + i));
        if (!e) break;

        out->items[i].tsMs = e->ts_ms;
        out->items[i].code = (int16_t)e->code;
        out->items[i].a = e->a;
        out->items[i].b = e->b;
    }
}


static void PlcCmd_Apply(const PlcCmd* cmd, PlcReply* rep)
{
    memset(rep, 0, sizeof(*rep));
    rep->ok = false;

    switch (cmd->type) {
        case PLC_CMD_UPLOAD_GRAPH:
            g_stagingGraph = cmd->u.upload.graph;
            g_stagingGraphValid = true;
            plc_log_put(PLC_EVT_UPLOAD_OK, (int16_t)g_stagingGraph.nodeCount, (int16_t)g_stagingGraph.cycleMs);
            rep->type = PLC_REPLY_UPLOAD_GRAPH_ACK;
            rep->ok = true;
            break;

        case PLC_CMD_ACTIVATE_GRAPH:
            rep->type = PLC_REPLY_ACTIVATE_GRAPH_ACK;
            rep->ok = plc_request_activate_graph();
            break;

        case PLC_CMD_STATUS:
            rep->type = PLC_REPLY_STATUS;
            rep->ok = true;
            fill_status_snapshot(&rep->u.status);
            break;

        case PLC_CMD_SAVE_ACTIVE:
            rep->type = PLC_REPLY_SAVE_ACTIVE_ACK;
            rep->ok = plc_persist_save();
            break;

        case PLC_CMD_LOAD_ACTIVE:
            rep->type = PLC_REPLY_LOAD_ACTIVE_ACK;
            g_needLoadActive = true;   // выполнить позже в safe-point
            rep->ok = true;            // ack сразу
            break;


        case PLC_CMD_LOG_DUMP:
            rep->type = PLC_REPLY_LOG_DUMP;
            rep->ok = true;
            fill_log_dump(&rep->u.logDump);
            break;

        case PLC_CMD_FORCE_OUTPUT:
            rep->type = PLC_REPLY_FORCE_OUTPUT_ACK;
            rep->ok = plc_force_output(cmd->u.force.nodeIndex, cmd->u.force.value, cmd->u.force.holdMs);
            break;

        case PLC_CMD_RELEASE_OUTPUT:
            rep->type = PLC_REPLY_RELEASE_OUTPUT_ACK;
            rep->ok = plc_release_output(cmd->u.release.nodeIndex);
            break;

        case PLC_CMD_MEM_READ:
            rep->type = PLC_REPLY_MEM_READ;
            rep->ok = true;
            fill_mem_read(cmd, &rep->u.memRead);
            break;

        case PLC_CMD_MEM_WRITE:
            rep->type = PLC_REPLY_MEM_WRITE_ACK;
            apply_mem_write(cmd, rep);
            break;

        case PLC_CMD_MEM_RESET:
            rep->type = PLC_REPLY_MEM_RESET_ACK;
            plc_mem_reset_all();
            rep->ok = true;
            rep->aux = UINT32_MAX;
            break;

        case PLC_CMD_MEM_INFO:
            rep->type = PLC_REPLY_MEM_INFO;
            rep->ok = true;
            fill_mem_info(&rep->u.memInfo);
            break;

        default:
            rep->type = PLC_REPLY_ERROR;
            rep->ok = false;
            break;
    }
}

static inline uint32_t cycles_to_us(uint32_t cycles)
{
    // SystemCoreClock должен быть актуален (обычно фиксированный на STM32)
    uint32_t hz = SystemCoreClock;
    if (hz == 0u) hz = 1u;
    // (cycles * 1_000_000) / hz
    return (uint32_t)((uint64_t)cycles * 1000000ull / (uint64_t)hz);
}


uint32_t PlcMsg_PumpApply(uint32_t maxItems, uint32_t timeoutMsFirst, uint32_t budgetUs)
{
    if (!s_plcQ) return 0u;
    if (maxItems == 0u) maxItems = 1u;

    uint32_t processed = 0;
    uint32_t t0 = DWT->CYCCNT;

    for (uint32_t i = 0; i < maxItems; i++) {
        if (budgetUs) {
            uint32_t dt = cycles_to_us((uint32_t)(DWT->CYCCNT - t0));
            if (dt >= budgetUs) break;
        }

        PlcQItem qi;
        uint32_t timeout = (processed == 0u) ? timeoutMsFirst : 0u;
        if (osMessageQueueGet(s_plcQ, &qi, NULL, timeout) != osOK) break;

        if (qi.slotIndex < PLC_RPC_SLOTS_MAX) {
            PlcRpcSlot* s = &s_slots[qi.slotIndex];
            PlcCmd_Apply(&s->cmd, &s->reply);
            (void)osEventFlagsSet(s->ev, 0x1u);
            processed++;
        }
    }
    return processed;
}

// ====== CBOR LOGGING HELPERS ======
#ifndef CBOR_LOG_ENABLED
#define CBOR_LOG_ENABLED 0
#endif

#ifndef CBOR_DUMP_MAX
#define CBOR_DUMP_MAX 1024u  // максимум байт входного буфера для hexdump (безопасно)
#endif

#ifndef MEM_RW_MAX_COUNT
#define MEM_RW_MAX_COUNT 512u   // верхний предел на одну команду, чтобы не убить MCU
#endif

// IEEE-754 half-precision (16-bit) → single-precision (32-bit) float
// Без UB, совместимо с -O2/-Os, работает на всех Cortex-M
static float half2float(uint16_t h) {
    // Битовые поля half-float: 1 sign | 5 exp | 10 mant
    const uint32_t sign = (h >> 15) & 1U;
    const uint32_t exp  = (h >> 10) & 0x1FU;
    const uint32_t mant = h & 0x3FFU;

    // Собираем 32-битное представление float по IEEE-754
    uint32_t f_bits;

    if (exp == 0x1FU) {
        // Inf / NaN
        f_bits = (sign << 31) | (0xFFU << 23); // exp=255
        if (mant != 0) {
            // NaN: включаем бит quiet-NaN (22-й)
            f_bits |= (1U << 22) | mant;
        }
        // else → ±Inf
    } else if (exp == 0) {
        // Zero или subnormal
        if (mant == 0) {
            // ±0.0
            f_bits = sign << 31;
        } else {
            // subnormal → normal (slow path, но корректный)
            // Простой способ: сдвигаем мантиссу и уменьшаем экспоненту
            // h = (-1)^s * 2^(-14) * (0.mant)
            // => f = (-1)^s * 2^(-126) * (0.mant << (112)) — но проще через float
            // Возвращаем через union, чтобы избежать UB
            union { float f; uint32_t u; } u;
            u.f = sign ? -0.0f : 0.0f;
            // Чтобы не усложнять — в большинстве embedded случаев subnormal = 0
            // Если нужна точность — можно добавить полную конверсию, но редко требуется
            // Здесь: округлим до 0 (приемлемо для датчиков и т.п.)
            return u.f;
        }
    } else {
        // normal number: exp_bias_half = 15, exp_bias_float = 127 → дельта = 112
        f_bits = (sign << 31) | ((exp + 112U) << 23) | (mant << 13);
    }

    // Безопасное преобразование битов → float (избегаем нарушения strict aliasing)
    union { uint32_t u; float f; } conv;
    conv.u = f_bits;
    return conv.f;
}

static const char* cbor_err_name(CborError e){
    switch(e){
        case CborNoError: return "CborNoError";
        case CborUnknownError: return "CborUnknownError";
        case CborErrorOutOfMemory: return "CborErrorOutOfMemory";
        case CborErrorAdvancePastEOF: return "CborErrorAdvancePastEOF";
        case CborErrorIO: return "CborErrorIO";
        case CborErrorUnexpectedEOF: return "CborErrorUnexpectedEOF";
        case CborErrorUnexpectedBreak: return "CborErrorUnexpectedBreak";
        case CborErrorUnknownType: return "CborErrorUnknownType";
        case CborErrorIllegalType: return "CborErrorIllegalType";
        case CborErrorIllegalNumber: return "CborErrorIllegalNumber";
        case CborErrorInvalidUtf8TextString: return "CborErrorInvalidUtf8TextString";
        case CborErrorTooManyItems: return "CborErrorTooManyItems";
        case CborErrorMapKeyNotString: return "CborErrorMapKeyNotString";
        case CborErrorMapNotSorted: return "CborErrorMapNotSorted";
        case CborErrorDataTooLarge: return "CborErrorDataTooLarge";
        case CborErrorNestingTooDeep: return "CborErrorNestingTooDeep";
        default: return "CborError(unknown)";
    }
}

static void cbor_log_val(const char* key, const CborValue* v){
#if CBOR_LOG_ENABLED
    CborType t = cbor_value_get_type(v);
    switch(t){
        case CborIntegerType:{
            int64_t x=0; CborError e=cbor_value_get_int64(v,&x);
            LOGT(tag, "  %s: type=int val=%lld (err=%s)", key, (long long)x, cbor_err_name(e));
        } break;
        case CborHalfFloatType: {
            uint16_t raw = 0;
            CborError e = cbor_value_get_half_float(v, &raw);
            float f = half2float(raw);
            LOGT(tag, "  %s: type=half raw=0x%04X -> %f (err=%s)",
                 key, (unsigned)raw, (double)f, cbor_err_name(e));
        } break;

        case CborFloatType: {
            float f=0; CborError e=cbor_value_get_float(v,&f);
            LOGT(tag, "  %s: type=float val=%f (err=%s)", key, (double)f, cbor_err_name(e));
        } break;

        case CborDoubleType: {
            double d=0; CborError e=cbor_value_get_double(v,&d);
            LOGT(tag, "  %s: type=double val=%f (err=%s)", key, d, cbor_err_name(e));
        } break;
        case CborTextStringType:{
            size_t l=0; CborError e=cbor_value_calculate_string_length(v,&l);
            LOGT(tag, "  %s: type=tstr len=%u (err=%s)", key, (unsigned)l, cbor_err_name(e));
        } break;
        case CborByteStringType:{
            size_t l=0; CborError e=cbor_value_calculate_string_length(v,&l);
            LOGT(tag, "  %s: type=bstr len=%u (err=%s)", key, (unsigned)l, cbor_err_name(e));
        } break;
        case CborArrayType: LOGT(tag, "  %s: type=array", key); break;
        case CborMapType:   LOGT(tag, "  %s: type=map",   key); break;
        case CborTagType:   LOGT(tag, "  %s: type=tag",   key); break;
        case CborSimpleType:LOGT(tag, "  %s: type=simple/null/bool/undef", key); break;
        default:            LOGT(tag, "  %s: type=invalid", key); break;
    }
#endif
}

static const char *cbor_type_name(CborType t) {
    switch (t) {
        case CborIntegerType:
            return "int";
        case CborByteStringType:
            return "bstr";
        case CborTextStringType:
            return "tstr";
        case CborArrayType:
            return "array";
        case CborMapType:
            return "map";
        case CborTagType:
            return "tag";
        case CborSimpleType:
            return "simple/null/undef/bool";
        case CborHalfFloatType:
            return "half";
        case CborFloatType:
            return "float";
        case CborDoubleType:
            return "double";
        case CborInvalidType:
        default:
            return "invalid";
    }
}

// ---- protocol counters ----
static uint32_t g_rx_frames = 0;
static uint32_t g_rx_crc_err = 0;
static uint32_t g_rx_parse_err = 0;
static uint32_t g_tx_frames = 0;

void proto_count_rx_frame_ok(void){ g_rx_frames++; }
void proto_count_rx_crc_err(void){ g_rx_crc_err++; }
void proto_count_rx_parse_err(void){ g_rx_parse_err++; }
void proto_count_tx_frame(void){ g_tx_frames++; }

extern const PlcLogEntry* plc_log_get_by_index(uint16_t idx);

// helper
static inline bool map_find(const CborValue *mapCont, const char *key, CborValue *out) {
    CborValue it = *mapCont; // копия итератора
    CborError e = cbor_value_map_find_value(&it, key, out);
    if (e != CborNoError) {
        LOGT(tag, "map_find('%s'): err=%s", key, cbor_err_name(e));
        return false;
    }
    if (!cbor_value_is_valid(out) || cbor_value_is_undefined(out)) {
        LOGT(tag, "map_find('%s'): not found / undefined", key);
        return false;
    }
    cbor_log_val(key, out);
    return true;
}

static bool get_bool(CborValue* v, bool* out)
{
    if (!v || !out) return false;
    if (!cbor_value_is_boolean(v)) return false;
    bool b = false;
    if (cbor_value_get_boolean(v, &b) != CborNoError) return false;
    *out = b;
    return true;
}

static bool get_int32(const CborValue *val, int32_t *out) {
    if (!cbor_value_is_integer(val)) return false;
    int64_t tmp;
    CborError err = cbor_value_get_int64(val, &tmp);
    if (err != CborNoError) return false;
    *out = (int32_t) tmp;
    return true;
}

static bool get_uint32(const CborValue *val, uint32_t *out) {
    if (!cbor_value_is_unsigned_integer(val) &&
        !cbor_value_is_integer(val))
        return false;
    uint64_t tmp;
    CborError err = cbor_value_get_uint64(val, &tmp);
    if (err != CborNoError) return false;
    *out = (uint32_t) tmp;
    return true;
}

static bool get_float(const CborValue *val, float *out) {
    if (!(cbor_value_is_half_float(val) ||
          cbor_value_is_float(val) ||
          cbor_value_is_double(val))) {
        return false;
    }
    float tmp;
    CborError err = cbor_value_get_float(val, &tmp); // get_double умеет и half/float
    if (err != CborNoError) return false;
    *out = (float) tmp;
    return true;
}

// Расширенная версия get_float_any с поддержкой half-float
static bool get_float_any(const CborValue *val, float *out) {
    if (!val || !out) {
        LOGT(tag, "ERR: get_float_any: NULL input");
        return false;
    }

    CborType t = cbor_value_get_type(val);
    LOGT(tag, "DEBUG: CBOR value type = %d", (int)t);

    if (t == CborHalfFloatType) {
        uint16_t h_raw;  // ← ВАЖНО: uint16_t, НЕ float!
        CborError e = cbor_value_get_half_float(val, &h_raw);
        if (e != CborNoError) {
            LOGT(tag, "ERR: cbor_value_get_half_float failed, error=%d", (int)e);
            return false;
        }

        float f_conv = half2float(h_raw);
        LOGT(tag, "DEBUG: half-float raw=0x%04X → decoded=%f", h_raw, f_conv);

        // Проверка: если получили 0, но сырое значение ≠ 0 — предупреждение
        if (f_conv == 0.0f && h_raw != 0x0000 && h_raw != 0x8000) {
            LOGT(tag, "WARN: half-float 0x%04X decoded as 0.0f — possible subnormal/NAN?", h_raw);
        }

        *out = f_conv;
        return true;
    }

    if (t == CborFloatType) {
        float f;
        CborError e = cbor_value_get_float(val, &f);
        if (e != CborNoError) {
            LOGT(tag, "ERR: cbor_value_get_float failed, error=%d", (int)e);
            return false;
        }
        LOGT(tag, "DEBUG: float32 decoded = %f", f);
        *out = f;
        return true;
    }

    if (t == CborDoubleType) {
        double d;
        CborError e = cbor_value_get_double(val, &d);
        if (e != CborNoError) {
            LOGT(tag, "ERR: cbor_value_get_double failed, error=%d", (int)e);
            return false;
        }
        float f = (float)d;
        LOGT(tag, "DEBUG: double → float: %f → %f", d, f);
        *out = f;
        return true;
    }

    if (t == CborIntegerType) {
        int64_t v;
        CborError e = cbor_value_get_int64(val, &v);
        if (e != CborNoError) {
            LOGT(tag, "ERR: cbor_value_get_int64 failed, error=%d", (int)e);
            return false;
        }
        float f = (float)v;
        LOGT(tag, "DEBUG: int64=%lld → float=%f", (long long)v, f);
        *out = f;
        return true;
    }

    LOGT(tag, "ERR: unsupported CBOR type %d for float conversion", (int)t);
    return false;
}

static CborError decode_single_node(const CborValue *nodeMapIt, PlcNode *outNode) {
    // Инициируем дефолты (на случай отсутствующих ключей)
    outNode->id = 0;
    outNode->type = 0;
    outNode->valueType = 0;
    outNode->inA = -1;
    outNode->inB = -1;
    outNode->paramInt = 0;
    outNode->paramFloat = 0.0f;
    outNode->paramMs = 0;
    outNode->flags = 0;

    if (!cbor_value_is_map(nodeMapIt)) {
        LOGT(tag, "node is not map, type=%s", cbor_type_name(cbor_value_get_type(nodeMapIt)));
        return CborErrorIllegalType;
    }

    CborValue val;

    // id
    if (map_find(nodeMapIt, "id", &val)) {
        LOGT(tag, "id type=%s", cbor_type_name(cbor_value_get_type(&val)));
        int32_t tmp;
        if (!get_int32(&val, &tmp)) {
            LOGT(tag, "id parse error");
            return CborErrorIllegalType;
        }
        outNode->id = tmp;
    } else {
        LOGT(tag, "id not present");
    }


    // type
    if (map_find(nodeMapIt, "type", &val)) {
        LOGT(tag, "type type=%s", cbor_type_name(cbor_value_get_type(&val)));
        int32_t tmp;
        if (!get_int32(&val, &tmp)) {
            LOGT(tag, "type parse error");
            return CborErrorIllegalType;
        }
        outNode->type = tmp;
    } else {
        LOGT(tag, "type not present");
    }

    // valueType
    if (map_find(nodeMapIt, "valueType", &val)) {
        LOGT(tag, "valueType type=%s", cbor_type_name(cbor_value_get_type(&val)));
        int32_t tmp;
        if (!get_int32(&val, &tmp)) {
            LOGT(tag, "valueType parse error");
            return CborErrorIllegalType;
        }
        outNode->valueType = tmp;
    } else {
        LOGT(tag, "valueType not present");
    }

    // inA
    if (map_find(nodeMapIt, "inA", &val)) {
        LOGT(tag, "inA type=%s", cbor_type_name(cbor_value_get_type(&val)));
        int32_t tmp;
        if (!get_int32(&val, &tmp)) {
            LOGT(tag, "inA parse error");
            return CborErrorIllegalType;
        }
        outNode->inA = (int16_t) tmp;
    } else {
        LOGT(tag, "inA not present");
    }

    // inB
    if (map_find(nodeMapIt, "inB", &val)) {
        LOGT(tag, "inB type=%s", cbor_type_name(cbor_value_get_type(&val)));
        int32_t tmp;
        if (!get_int32(&val, &tmp)) {
            LOGT(tag, "inB parse error");
            return CborErrorIllegalType;
        }
        outNode->inB = (int16_t) tmp;
    } else {
        LOGT(tag, "inB not present");
    }

    // paramInt
    if (map_find(nodeMapIt, "paramInt", &val)) {
        LOGT(tag, "paramInt type=%s", cbor_type_name(cbor_value_get_type(&val)));
        int32_t tmp;
        if (!get_int32(&val, &tmp)) {
            LOGT(tag, "paramInt parse error");
            return CborErrorIllegalType;
        }
        outNode->paramInt =  tmp;
    } else {
        LOGT(tag, "paramInt not present");
    }

    // paramFloat
    if (map_find(nodeMapIt, "paramFloat", &val)) {
        LOGT(tag, "paramFloat type=%s", cbor_type_name(cbor_value_get_type(&val)));
        float tmpf;
        if (!get_float_any(&val, &tmpf)) {
            LOGT(tag, "paramFloat parse error");
            return CborErrorIllegalType;
        }
        LOGT(tag, "paramFloat=%.6f", tmpf);
        outNode->paramFloat = tmpf;
    } else {
        LOGT(tag, "paramFloat not present");
    }

    // sanity: если HYST и paramFloat==0, а paramMs!=0 — подсказка
    if(outNode->type == PLC_NODE_HYST){
        if(outNode->paramFloat == 0.0f && outNode->paramMs != 0){
            LOGT(tag, "WARN: HYST center float=0.0, but paramMs=%u (consider fallback center=paramMs)", (unsigned)outNode->paramMs);
        }
    }

    // paramMs
    if (map_find(nodeMapIt, "paramMs", &val)) {
        LOGT(tag, "paramMs type=%s", cbor_type_name(cbor_value_get_type(&val)));
        uint32_t tmpu;
        if (!get_uint32(&val, &tmpu)) {
            LOGT(tag, "paramMs parse error");
            return CborErrorIllegalType;
        }
        outNode->paramMs = tmpu;
    } else {
        LOGT(tag, "paramMs not present");
    }

    // flags
    if (map_find(nodeMapIt, "flags", &val)) {
        LOGT(tag, "flags type=%s", cbor_type_name(cbor_value_get_type(&val)));
        uint32_t tmpu;
        if (!get_uint32(&val, &tmpu)) {
            LOGT(tag, "flags parse error");
            return CborErrorIllegalType;
        }
        outNode->flags = tmpu;
    } else {
        LOGT(tag, "flags not present");
    }

    LOGT(tag, "node summary: id=%d type=%d vtype=%d inA=%d inB=%d pInt=%d pF=%f pMs=%u flags=0x%08X",
         (int)outNode->id, (int)outNode->type, (int)outNode->valueType,
         (int)outNode->inA, (int)outNode->inB, (int)outNode->paramInt,
         (double)outNode->paramFloat, (unsigned)outNode->paramMs, (unsigned)outNode->flags);

    if (outNode->type == PLC_NODE_HYST){
        float h = (float)outNode->paramInt * 0.001f; if(h<0) h=-h;
        float lo = outNode->paramFloat - 0.5f*h;
        float hi = outNode->paramFloat + 0.5f*h;
        LOGT(tag, "HYST params: center=%f width=%f -> lo=%f hi=%f",
             (double)outNode->paramFloat, (double)h, (double)lo, (double)hi);
    }

    return CborNoError;
}

static CborError decode_graph(const CborValue *graphMapIt, PlcGraph *outGraph) {
    if (!cbor_value_is_map(graphMapIt)) {
        LOGT(tag, "graph is not map, type=%s", cbor_type_name(cbor_value_get_type(graphMapIt)));
        return CborErrorIllegalType;
    }

    outGraph->cycleMs = 10;
    outGraph->nodeCount = 0;

    CborValue val;

    // cycleMs
    if (map_find(graphMapIt, "cycleMs", &val)) {
        uint32_t cyc;
        if (!get_uint32(&val, &cyc)) {
            LOGT(tag, "cycleMs parse error");
            return CborErrorIllegalType;
        }
        outGraph->cycleMs = cyc;
    } else {
        LOGT(tag, "cycleMs not present (default=10)");
    }

    // nodes[]
    if (!map_find(graphMapIt, "nodes", &val)) {
        LOGT(tag, "nodes missing");
        return CborErrorUnknownType;
    }
    if (!cbor_value_is_array(&val)) {
        LOGT(tag, "nodes is not array, type=%s", cbor_type_name(cbor_value_get_type(&val)));
        return CborErrorIllegalType;
    }

    LOGT(tag, "enter nodes array");
    CborValue arrIt;
    CborError err = cbor_value_enter_container(&val, &arrIt);
    if (err != CborNoError) {
        LOGT(tag, "enter_container(nodes) err=%s", cbor_err_name(err));
        return err;
    }

    uint16_t count = 0;
    while (!cbor_value_at_end(&arrIt)) {
        if (count >= PLC_MAX_NODES) {
            LOGT(tag, "Too many nodes: %u >= PLC_MAX_NODES", (unsigned)count);
            return CborErrorOutOfMemory;
        }
        if (!cbor_value_is_map(&arrIt)) {
            LOGT(tag, "nodes[%u] not a map, type=%s", (unsigned)count, cbor_type_name(cbor_value_get_type(&arrIt)));
            return CborErrorIllegalType;
        }

        CborValue nodeMapIt = arrIt; // копия для decode
        err = decode_single_node(&nodeMapIt, &outGraph->nodes[count]);
        if (err != CborNoError) {
            LOGT(tag, "decode_single_node[%u] err=%s", (unsigned)count, cbor_err_name(err));
            return err;
        }

        err = cbor_value_advance(&arrIt);
        if (err != CborNoError) {
            LOGT(tag, "advance nodes[%u] err=%s", (unsigned)count, cbor_err_name(err));
            return err;
        }
        count++;
    }

    outGraph->nodeCount = count;
    LOGT(tag, "leave nodes array, count=%u", (unsigned)count);

    err = cbor_value_leave_container(&val, &arrIt);
    if (err != CborNoError) {
        LOGT(tag, "leave_container(nodes) err=%s", cbor_err_name(err));
    }
    return err;
}

bool CborProto_DecodeCmd(const uint8_t* buf, size_t len, PlcCmd* outCmd)
{
    if (!buf || !len || !outCmd) return false;
    memset(outCmd, 0, sizeof(*outCmd));

    CborParser parser;
    CborValue root;
    CborError err = cbor_parser_init(buf, len, 0, &parser, &root);
    if (err != CborNoError) { proto_count_rx_parse_err(); return false; }

    if (!cbor_value_is_map(&root)) { proto_count_rx_parse_err(); return false; }

    CborValue valCmd;
    if(!map_find(&root, "cmd", &valCmd) || !cbor_value_is_text_string(&valCmd)) { proto_count_rx_parse_err(); return false; }

    char cmdBuf[32]; size_t cmdLen = sizeof(cmdBuf) - 1;
    err = cbor_value_copy_text_string(&valCmd, cmdBuf, &cmdLen, NULL);
    if (err != CborNoError) { proto_count_rx_parse_err(); return false; }
    cmdBuf[cmdLen] = '\0';

    if (strcmp(cmdBuf, "upload_graph") == 0) {
        CborValue valGraph;
        if (!map_find(&root, "graph", &valGraph) || !cbor_value_is_map(&valGraph)) return false;
        outCmd->type = PLC_CMD_UPLOAD_GRAPH;
        memset(&outCmd->u.upload.graph, 0, sizeof(PlcGraph));
        if (decode_graph(&valGraph, &outCmd->u.upload.graph) != CborNoError) return false;
        return true;
    }

    if (strcmp(cmdBuf, "activate_graph") == 0) { outCmd->type = PLC_CMD_ACTIVATE_GRAPH; return true; }
    if (strcmp(cmdBuf, "status") == 0)         { outCmd->type = PLC_CMD_STATUS; return true; }
    if (strcmp(cmdBuf, "save_active") == 0)    { outCmd->type = PLC_CMD_SAVE_ACTIVE; return true; }
    if (strcmp(cmdBuf, "load_active") == 0)    { outCmd->type = PLC_CMD_LOAD_ACTIVE; return true; }
    if (strcmp(cmdBuf, "log_dump") == 0)       { outCmd->type = PLC_CMD_LOG_DUMP; return true; }

    if (strcmp(cmdBuf, "force_output") == 0) {
        CborValue v; int32_t nodeIdx32, value32; uint32_t holdMs = 0;
        if (!map_find(&root, "nodeIndex", &v) || !get_int32(&v, &nodeIdx32)) return false;
        if (!map_find(&root, "value", &v) || !get_int32(&v, &value32)) return false;
        if (map_find(&root, "holdMs", &v)) { if (!get_uint32(&v, &holdMs)) return false; }
        if (nodeIdx32 < 0 || nodeIdx32 >= PLC_MAX_NODES) return false;

        outCmd->type = PLC_CMD_FORCE_OUTPUT;
        outCmd->u.force.nodeIndex = (uint16_t)nodeIdx32;
        outCmd->u.force.value = (value32 != 0);
        outCmd->u.force.holdMs = holdMs;
        return true;
    }

    if (strcmp(cmdBuf, "release_output") == 0) {
        CborValue v; int32_t nodeIdx32;
        if (!map_find(&root, "nodeIndex", &v) || !get_int32(&v, &nodeIdx32)) return false;
        if (nodeIdx32 < 0 || nodeIdx32 >= PLC_MAX_NODES) return false;
        outCmd->type = PLC_CMD_RELEASE_OUTPUT;
        outCmd->u.release.nodeIndex = (uint16_t)nodeIdx32;
        return true;
    }

    // mem_read/mem_write/mem_reset/mem_info: тип/диапазоны совпадают с твоими handlers
    if (strcmp(cmdBuf, "mem_read") == 0) {
        // reuse твою map_find/get_uint32/get_text_string helpers
        CborValue vType, vFrom, vCount;
        char typeBuf[8]; size_t tl = sizeof(typeBuf)-1;
        uint32_t from=0, count=0;

        if (!map_find(&root, "type", &vType) || !cbor_value_is_text_string(&vType)) return false;
        if (cbor_value_copy_text_string(&vType, typeBuf, &tl, NULL) != CborNoError) return false;
        typeBuf[tl]='\0';
        if (!map_find(&root, "from", &vFrom) || !get_uint32(&vFrom, &from)) return false;
        if (!map_find(&root, "count", &vCount) || !get_uint32(&vCount, &count)) return false;

        PlcMemType mt = 0;
        if (strcmp(typeBuf,"bool")==0) mt = PLC_MEM_T_BOOL;
        else if (strcmp(typeBuf,"int")==0) mt = PLC_MEM_T_INT;
        else if (strcmp(typeBuf,"real")==0) mt = PLC_MEM_T_REAL;
        else return false;

        outCmd->type = PLC_CMD_MEM_READ;
        outCmd->u.memRead.memType = mt;
        outCmd->u.memRead.from = (uint16_t)from;
        outCmd->u.memRead.count = (uint16_t)count;
        return true;
    }

    if (strcmp(cmdBuf, "mem_write") == 0) {
        // Ограничиваем count размерами PLC_MEM_*_COUNT, чтобы PlcCmd был фиксированный.
        // values[] читаем в фикс массив (32).
        CborValue vType, vFrom, vArr;
        char typeBuf[8]; size_t tl = sizeof(typeBuf)-1;
        uint32_t from=0;

        if (!map_find(&root, "type", &vType) || !cbor_value_is_text_string(&vType)) return false;
        if (cbor_value_copy_text_string(&vType, typeBuf, &tl, NULL) != CborNoError) return false;
        typeBuf[tl]='\0';
        if (!map_find(&root, "from", &vFrom) || !get_uint32(&vFrom, &from)) return false;
        if (!map_find(&root, "values", &vArr) || !cbor_value_is_array(&vArr)) return false;

        PlcMemType mt = 0;
        uint16_t maxN = 0;
        if (strcmp(typeBuf,"bool")==0) { mt = PLC_MEM_T_BOOL; maxN = PLC_MEM_BOOL_COUNT; }
        else if (strcmp(typeBuf,"int")==0) { mt = PLC_MEM_T_INT; maxN = PLC_MEM_INT_COUNT; }
        else if (strcmp(typeBuf,"real")==0) { mt = PLC_MEM_T_REAL; maxN = PLC_MEM_REAL_COUNT; }
        else return false;

        outCmd->type = PLC_CMD_MEM_WRITE;
        outCmd->u.memWrite.memType = mt;
        outCmd->u.memWrite.from = (uint16_t)from;

        // iterate array
        CborValue it;
        if (cbor_value_enter_container(&vArr, &it) != CborNoError) return false;

        uint16_t n = 0;
        while (!cbor_value_at_end(&it) && n < maxN) {
            if (mt == PLC_MEM_T_BOOL) {
                bool vb=false;
                if (!get_bool(&it, &vb)) return false;
                outCmd->u.memWrite.v.b[n] = vb;
            } else if (mt == PLC_MEM_T_INT) {
                int32_t vi=0;
                if (!get_int32(&it, &vi)) return false;
                outCmd->u.memWrite.v.i[n] = vi;
            } else {
                float vf=0;
                if (!get_float(&it, &vf)) return false;
                outCmd->u.memWrite.v.f[n] = vf;
            }
            n++;
            if (cbor_value_advance(&it) != CborNoError) break;
        }

        outCmd->u.memWrite.count = n;
        (void)cbor_value_leave_container(&vArr, &it);
        return true;
    }

    if (strcmp(cmdBuf, "mem_reset") == 0) { outCmd->type = PLC_CMD_MEM_RESET; return true; }
    if (strcmp(cmdBuf, "mem_info") == 0)  { outCmd->type = PLC_CMD_MEM_INFO; return true; }

    return false;
}

bool CborProto_EncodeReply(const PlcReply* r, CborReply* out)
{
    if (!r || !out) return false;
    out->len = 0;

    CborEncoder root, map, arr, nmap;
    cbor_encoder_init(&root, out->data, sizeof(out->data), 0);
    if (cbor_encoder_create_map(&root, &map, CborIndefiniteLength) != CborNoError) return false;

    // cmd string (совместимо с твоими *_ack)
    const char* cmd = "error";
    switch (r->type) {
        case PLC_REPLY_UPLOAD_GRAPH_ACK:   cmd = "upload_graph_ack"; break;
        case PLC_REPLY_ACTIVATE_GRAPH_ACK: cmd = "activate_graph_ack"; break;
        case PLC_REPLY_SAVE_ACTIVE_ACK:    cmd = "save_active_ack"; break;
        case PLC_REPLY_LOAD_ACTIVE_ACK:    cmd = "load_active_ack"; break;
        case PLC_REPLY_FORCE_OUTPUT_ACK:   cmd = "force_output_ack"; break;
        case PLC_REPLY_RELEASE_OUTPUT_ACK: cmd = "release_output_ack"; break;
        case PLC_REPLY_MEM_WRITE_ACK:      cmd = "mem_write_ack"; break;
        case PLC_REPLY_MEM_RESET_ACK:      cmd = "mem_reset_ack"; break;
        case PLC_REPLY_STATUS:             cmd = "status"; break;
        case PLC_REPLY_MEM_READ:           cmd = "mem_read"; break;
        case PLC_REPLY_MEM_INFO:           cmd = "mem_info"; break;
        case PLC_REPLY_LOG_DUMP:           cmd = "log_dump"; break;
        default:                           cmd = "error"; break;
    }

    cbor_encode_text_stringz(&map, "cmd");
    cbor_encode_text_stringz(&map, cmd);

//    cbor_encode_text_stringz(&map, "ok");
//    cbor_encode_boolean(&map, r->ok);

    // common "ok"
    if (r->type != PLC_REPLY_STATUS && r->type != PLC_REPLY_MEM_READ && r->type != PLC_REPLY_MEM_INFO && r->type != PLC_REPLY_LOG_DUMP) {
        cbor_encode_text_stringz(&map, "ok");
        cbor_encode_boolean(&map, r->ok);

        // optional aux
        if (r->type == PLC_REPLY_MEM_WRITE_ACK) {
            cbor_encode_text_stringz(&map, "written");
            cbor_encode_uint(&map, r->aux);
        }
    }

    if (r->type == PLC_REPLY_STATUS) {
        const PlcStatusSnapshot* s = &r->u.status;

        cbor_encode_text_stringz(&map, "uptimeMs");
        cbor_encode_uint(&map, s->uptimeMs);

        cbor_encode_text_stringz(&map, "active");
        cbor_encode_boolean(&map, s->active);

        cbor_encode_text_stringz(&map, "cycleMs");
        cbor_encode_uint(&map, s->cycleMs);

        cbor_encode_text_stringz(&map, "scanTickAvgUs");
        cbor_encode_uint(&map, s->scanTickAvgUs);
        cbor_encode_text_stringz(&map, "scanTickMaxUs");
        cbor_encode_uint(&map, s->scanTickMaxUs);

        cbor_encode_text_stringz(&map, "scanWorkAvgUs");
        cbor_encode_uint(&map, s->scanWorkAvgUs);
        cbor_encode_text_stringz(&map, "scanWorkMaxUs");
        cbor_encode_uint(&map, s->scanWorkMaxUs);

        cbor_encode_text_stringz(&map, "cycleRealAvgMs");
        cbor_encode_uint(&map, s->cycleRealAvgMs);
        cbor_encode_text_stringz(&map, "cycleRealMaxMs");
        cbor_encode_uint(&map, s->cycleRealMaxMs);

        // совместимость: старые поля scanAvgUs/scanMaxUs как "work"
        cbor_encode_text_stringz(&map, "scanAvgUs");
        cbor_encode_uint(&map, s->scanWorkAvgUs);
        cbor_encode_text_stringz(&map, "scanMaxUs");
        cbor_encode_uint(&map, s->scanWorkMaxUs);

        cbor_encode_text_stringz(&map, "scanAvgMs");
        cbor_encode_uint(&map, s->scanTickAvgUs / 1000u);
        cbor_encode_text_stringz(&map, "scanMaxMs");
        cbor_encode_uint(&map, s->scanTickMaxUs / 1000u);

        cbor_encode_text_stringz(&map, "scanLongSteps");
        cbor_encode_uint(&map, s->scanLongSteps);

        cbor_encode_text_stringz(&map, "nodesCount");
        cbor_encode_uint(&map, s->nodesCount);

        cbor_encode_text_stringz(&map, "nodesState");
        cbor_encoder_create_array(&map, &arr, s->nodesCount);

        for (uint16_t i = 0; i < s->nodesCount && i < PLC_MAX_NODES; i++) {
            const PlcNodeSnapshot* n = &s->nodes[i];
            cbor_encoder_create_map(&arr, &nmap, CborIndefiniteLength);

            cbor_encode_text_stringz(&nmap, "id"); cbor_encode_uint(&nmap, n->id);
            cbor_encode_text_stringz(&nmap, "type"); cbor_encode_uint(&nmap, n->type);
            cbor_encode_text_stringz(&nmap, "valueType"); cbor_encode_uint(&nmap, n->valueType);

            if (n->valueType == PLC_VAL_BOOL) { cbor_encode_text_stringz(&nmap, "outBool"); cbor_encode_boolean(&nmap, n->out.b); }
            else if (n->valueType == PLC_VAL_INT) { cbor_encode_text_stringz(&nmap, "outInt"); cbor_encode_int(&nmap, n->out.i); }
            else { cbor_encode_text_stringz(&nmap, "outFloat"); cbor_encode_float(&nmap, n->out.f); }

            if (n->tonMs) { cbor_encode_text_stringz(&nmap, "tonMs"); cbor_encode_uint(&nmap, n->tonMs); }
            if (n->toffLeftMs) { cbor_encode_text_stringz(&nmap, "toffLeftMs"); cbor_encode_uint(&nmap, n->toffLeftMs); }

            if (n->type == PLC_NODE_PID) {
                cbor_encode_text_stringz(&nmap, "sp"); cbor_encode_float(&nmap, n->sp);
                cbor_encode_text_stringz(&nmap, "pv"); cbor_encode_float(&nmap, n->pv);
                cbor_encode_text_stringz(&nmap, "i");  cbor_encode_float(&nmap, n->i_term);
                cbor_encode_text_stringz(&nmap, "u");  cbor_encode_float(&nmap, n->u);
            }

            if (n->forceActive) {
                cbor_encode_text_stringz(&nmap, "forceActive"); cbor_encode_boolean(&nmap, true);
                cbor_encode_text_stringz(&nmap, "forceValue");  cbor_encode_boolean(&nmap, n->forceValue);
                cbor_encode_text_stringz(&nmap, "forceLeftMs"); cbor_encode_uint(&nmap, n->forceLeftMs);
            }

            cbor_encoder_close_container(&arr, &nmap);
        }
        cbor_encoder_close_container(&map, &arr);
    }

    if (r->type == PLC_REPLY_MEM_INFO) {
        cbor_encode_text_stringz(&map, "boolCount"); cbor_encode_uint(&map, r->u.memInfo.boolCount);
        cbor_encode_text_stringz(&map, "intCount");  cbor_encode_uint(&map, r->u.memInfo.intCount);
        cbor_encode_text_stringz(&map, "realCount"); cbor_encode_uint(&map, r->u.memInfo.realCount);
    }

    if (r->type == PLC_REPLY_MEM_READ) {
        const PlcMemReadSnapshot* m = &r->u.memRead;

        const char* t = (m->memType==PLC_MEM_T_BOOL)?"bool":(m->memType==PLC_MEM_T_INT)?"int":"real";
        cbor_encode_text_stringz(&map, "type");  cbor_encode_text_stringz(&map, t);
        cbor_encode_text_stringz(&map, "from");  cbor_encode_uint(&map, m->from);
        cbor_encode_text_stringz(&map, "count"); cbor_encode_uint(&map, m->count);

        cbor_encode_text_stringz(&map, "values");
        cbor_encoder_create_array(&map, &arr, m->count);

        for (uint16_t i=0; i<m->count; i++) {
            if (m->memType==PLC_MEM_T_BOOL) cbor_encode_boolean(&arr, m->v.b[i]);
            else if (m->memType==PLC_MEM_T_INT) cbor_encode_int(&arr, m->v.i[i]);
            else cbor_encode_float(&arr, m->v.f[i]);
        }

        cbor_encoder_close_container(&map, &arr);
    }

    if (r->type == PLC_REPLY_LOG_DUMP) {
        cbor_encode_text_stringz(&map, "count");
        cbor_encode_uint(&map, r->u.logDump.count);

        cbor_encode_text_stringz(&map, "items");
        cbor_encoder_create_array(&map, &arr, r->u.logDump.count);

        for (uint16_t i=0; i<r->u.logDump.count; i++) {
            const PlcLogItemSnapshot* e = &r->u.logDump.items[i];
            cbor_encoder_create_map(&arr, &nmap, 4);
            cbor_encode_text_stringz(&nmap, "tsMs"); cbor_encode_uint(&nmap, e->tsMs);
            cbor_encode_text_stringz(&nmap, "code"); cbor_encode_int(&nmap, e->code);
            cbor_encode_text_stringz(&nmap, "a");    cbor_encode_int(&nmap, e->a);
            cbor_encode_text_stringz(&nmap, "b");    cbor_encode_int(&nmap, e->b);
            cbor_encoder_close_container(&arr, &nmap);
        }
        cbor_encoder_close_container(&map, &arr);
    }

    cbor_encoder_close_container(&root, &map);
    out->len = cbor_encoder_get_buffer_size(&root, out->data);
    proto_count_tx_frame();
    return true;
}




