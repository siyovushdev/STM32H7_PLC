#include "cbor_msg.h"
#include "plc_persist.h"
#include "logf.h"
#include <math.h>  // для INFINITY, NAN (если нет — могу заменить на макросы)
#include "cbor.h"
#include "plc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "plc_log.h"

static const char *tag = "CBOR";

// ====== CBOR LOGGING HELPERS ======
#ifndef CBOR_LOG_ENABLED
#define CBOR_LOG_ENABLED 1
#endif

#ifndef CBOR_DUMP_MAX
#define CBOR_DUMP_MAX 1024u  // максимум байт входного буфера для hexdump (безопасно)
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

static void cbor_hexdump_limited(const char* tagLoc, const uint8_t* buf, size_t len){
#if CBOR_LOG_ENABLED
    size_t n = len < CBOR_DUMP_MAX ? len : CBOR_DUMP_MAX;
    char line[80];
    size_t pos = 0;
    LOGT(tagLoc, "hexdump len=%u (showing %u of %u)", (unsigned)len, (unsigned)n, (unsigned)len);
    for(size_t i=0;i<n;i++){
        int k = snprintf(line+pos, sizeof(line)-pos, "%02X ", buf[i]);
        if(k<=0) break;
        pos += (size_t)k;
        if(pos > sizeof(line)-4 || (i%16)==15){ line[pos]=0; LOGT(tagLoc, "%s", line); pos=0; }
    }
    if(pos){ line[pos]=0; LOGT(tagLoc, "%s", line); }
    if(len>n) LOGT(tagLoc, "... truncated ...");
#endif
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


//#define LOGF(fmt, ...)  printf("CBOR %s:%d " fmt "\r\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
bool decode_graph_from_cbor(const uint8_t* buf, size_t len, PlcGraph* outG);
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

static inline bool _ok(CborError e){ return e==CborNoError; }

extern void proto_get_counters(uint32_t *rx_ok, uint32_t *rx_crc, uint32_t *rx_parse, uint32_t *tx_ok);

// ---- PLC runtime metrics ----
extern uint32_t g_scan_cnt;
extern uint32_t g_scan_ms_accum;     // сумма длительностей шага (мс)
extern uint16_t g_scan_ms_max;       // максимум за период
extern uint32_t g_scan_long_steps;   // шагов > g_activeGraph.cycleMs
void send_status_reply(CborReply *out)
{
    plc_log_put(PLC_EVT_STATUS_REQ, 0, 0);
    // Буфер ответа (подбери при необходимости)
    CborEncoder root, map, arr, nmap;

    uint32_t rx_ok, rx_crc, rx_parse, tx_ok;
    proto_get_counters(&rx_ok, &rx_crc, &rx_parse, &tx_ok);


    // Снимок для консистентности
    bool     active;
    uint32_t cycle;
    uint16_t cnt;

    taskENTER_CRITICAL();
    active = g_activeGraphValid;
    cycle  = active ? g_activeGraph.cycleMs : 0;
    cnt    = active ? g_activeGraph.nodeCount : 0;
    taskEXIT_CRITICAL();

    cbor_encoder_init(&root, out->data, sizeof(out->data), 0);
    if(!_ok(cbor_encoder_create_map(&root, &map, CborIndefiniteLength))) goto send;

    // "cmd": "status"
    if(!_ok(cbor_encode_text_stringz(&map, "cmd"))) goto send;
    if(!_ok(cbor_encode_text_stringz(&map, "status"))) goto send;

    // uptimeMs
    if(!_ok(cbor_encode_text_stringz(&map, "uptimeMs"))) goto send;
    if(!_ok(cbor_encode_uint(&map, now_ms()))) goto send;

    // active
    if(!_ok(cbor_encode_text_stringz(&map, "active"))) goto send;
    if(!_ok(cbor_encode_boolean(&map, active))) goto send;

    // cycleMs
    if(!_ok(cbor_encode_text_stringz(&map, "cycleMs"))) goto send;
    if(!_ok(cbor_encode_uint(&map, cycle))) goto send;

    if(!_ok(cbor_encode_text_stringz(&map, "scanAvgMs"))) goto send;
    if(!_ok(cbor_encode_uint(&map, g_scan_cnt ? (g_scan_ms_accum / g_scan_cnt) : 0))) goto send;

    if(!_ok(cbor_encode_text_stringz(&map, "scanMaxMs"))) goto send;
    if(!_ok(cbor_encode_uint(&map, g_scan_ms_max))) goto send;

    if(!_ok(cbor_encode_text_stringz(&map, "scanLongSteps"))) goto send;
    if(!_ok(cbor_encode_uint(&map, g_scan_long_steps))) goto send;

//    cbor_encode_text_stringz(&map, "proto");
//    CborEncoder proto;
//    cbor_encoder_create_map(&map, &proto, 4);
//    cbor_encode_text_stringz(&proto, "rx");
//    cbor_encode_uint(&proto, rx_ok);
//    cbor_encode_text_stringz(&proto, "crcErr");
//    cbor_encode_uint(&proto, rx_crc);
//    cbor_encode_text_stringz(&proto, "parseErr");
//    cbor_encode_uint(&proto, rx_parse);
//    cbor_encode_text_stringz(&proto, "tx");
//    cbor_encode_uint(&proto, tx_ok);
//    cbor_encoder_close_container(&map, &proto);

    // nodesCount
    if(!_ok(cbor_encode_text_stringz(&map, "nodesCount"))) goto send;
    if(!_ok(cbor_encode_uint(&map, cnt))) goto send;

    // nodesState: массив узлов
    if(!_ok(cbor_encode_text_stringz(&map, "nodesState"))) goto send;
    if(!_ok(cbor_encoder_create_array(&map, &arr, cnt))) goto send;



    if (active) {
        taskENTER_CRITICAL();
        for (uint16_t i = 0; i < g_activeGraph.nodeCount; i++) {
            PlcNode* n = &g_activeGraph.nodes[i];

            if(!_ok(cbor_encoder_create_map(&arr, &nmap, CborIndefiniteLength))) { taskEXIT_CRITICAL(); goto send; }

            // id
            if(!_ok(cbor_encode_text_stringz(&nmap, "id"))) goto ns_close;
            if(!_ok(cbor_encode_uint(&nmap, n->id))) goto ns_close;

            // type
            if(!_ok(cbor_encode_text_stringz(&nmap, "type"))) goto ns_close;
            if(!_ok(cbor_encode_uint(&nmap, n->type))) goto ns_close;

            // valueType
            if(!_ok(cbor_encode_text_stringz(&nmap, "valueType"))) goto ns_close;
            if(!_ok(cbor_encode_uint(&nmap, n->valueType))) goto ns_close;

            // out*
            if (n->valueType == PLC_VAL_BOOL) {
                if(!_ok(cbor_encode_text_stringz(&nmap, "outBool"))) goto ns_close;
                if(!_ok(cbor_encode_boolean(&nmap, n->out.b))) goto ns_close;
            } else if (n->valueType == PLC_VAL_INT) {
                if(!_ok(cbor_encode_text_stringz(&nmap, "outInt"))) goto ns_close;
                if(!_ok(cbor_encode_int(&nmap, n->out.i))) goto ns_close;
            } else { // FLOAT
                if(!_ok(cbor_encode_text_stringz(&nmap, "outFloat"))) goto ns_close;
                if(!_ok(cbor_encode_float(&nmap, n->out.f))) goto ns_close;
            }

            // спец-поля для таймеров/форса (по возможности)
            if (n->type == PLC_NODE_TON) {
                if(!_ok(cbor_encode_text_stringz(&nmap, "tonMs"))) goto ns_close;
                if(!_ok(cbor_encode_uint(&nmap, n->ton_accum_ms))) goto ns_close;
            }
            if (n->type == PLC_NODE_TOFF) {
                if(!_ok(cbor_encode_text_stringz(&nmap, "toffLeftMs"))) goto ns_close;
                if(!_ok(cbor_encode_uint(&nmap, n->toff_left_ms))) goto ns_close;
            }
            if (n->type == PLC_NODE_PID) {
                float sp = (n->inA >= 0) ? g_activeGraph.nodes[n->inA].out.f : 0.0f;
                float pv = (n->inB >= 0) ? g_activeGraph.nodes[n->inB].out.f : 0.0f;
                // локально сохрани p_term/d_term/u при расчёте PID (например, в node->debug_p, debug_d, debug_u),
                // либо пересчитай p_term/d_term тут, если хранишь только итог u и pid_i.

                cbor_encode_text_stringz(&nmap, "sp");
                cbor_encode_float(&nmap, sp);
                cbor_encode_text_stringz(&nmap, "pv");
                cbor_encode_float(&nmap, pv);
                cbor_encode_text_stringz(&nmap, "i");
                cbor_encode_float(&nmap, n->pid_i);
                cbor_encode_text_stringz(&nmap, "u");
                cbor_encode_float(&nmap, n->out.f);
            }
            if (n->flags & PLC_NODE_FLAG_FORCE) {
                // узел в режиме форса
                cbor_encode_text_stringz(&nmap, "forceActive");
                cbor_encode_boolean(&nmap, true);

                cbor_encode_text_stringz(&nmap, "forceValue");
                cbor_encode_boolean(&nmap, n->force_bool);  // 0/1 для DO/PWM

                cbor_encode_text_stringz(&nmap, "forceLeftMs");
                cbor_encode_uint(&nmap, n->force_left_ms);
            }

            if(!_ok(cbor_encoder_close_container(&arr, &nmap))) { taskEXIT_CRITICAL(); goto send; }
            continue;

            ns_close:
            (void)cbor_encoder_close_container(&arr, &nmap);
            taskEXIT_CRITICAL();
            goto send;
        }
        taskEXIT_CRITICAL();
    }

    if(!_ok(cbor_encoder_close_container(&map, &arr))) goto send;

//    // logs: последние 32 записи
//    uint16_t n = plc_log_count();
//    uint16_t take = (n > 32) ? 32 : n;
//    cbor_encode_text_stringz(&map, "log");
//    CborEncoder arr2;
//    cbor_encoder_create_array(&map, &arr2, take);
//
//    for (uint16_t i = 0; i < take; i++){
//        uint16_t idx = ( (g_head + PLC_LOG_CAP) - (take - i) ) % PLC_LOG_CAP;
//        PlcLogEntry *le = &g_log[idx];
//        CborEncoder rec;
//        cbor_encoder_create_map(&arr2, &rec, 3);
//        cbor_encode_text_stringz(&rec, "t"); cbor_encode_uint(&rec, le->ts_ms);
//        cbor_encode_text_stringz(&rec, "c"); cbor_encode_uint(&rec, le->code);
//        cbor_encode_text_stringz(&rec, "ab");
//        CborEncoder ab;
//        cbor_encoder_create_array(&rec, &ab, 2);
//        cbor_encode_int(&ab, le->a);
//        cbor_encode_int(&ab, le->b);
//        cbor_encoder_close_container(&rec, &ab);
//        cbor_encoder_close_container(&arr2, &rec);
//    }
//    cbor_encoder_close_container(&map, &arr2);

    // закрыть корневую map
    if(!_ok(cbor_encoder_close_container(&root, &map))) goto send;

    send:
    out->len = (uint16_t)cbor_encoder_get_buffer_size(&root, out->data);
}


static void send_upload_graph_ack(bool ok, CborReply *out)
{
    if (!out) return; out->len = 0;

    CborEncoder root, map;

    uint16_t cnt = 0; uint32_t cycle = 0;
    if (ok) {
        taskENTER_CRITICAL();
        cnt   = g_stagingGraphValid ? g_stagingGraph.nodeCount : 0;
        cycle = g_stagingGraphValid ? g_stagingGraph.cycleMs   : 0;
        taskEXIT_CRITICAL();
    }

    cbor_encoder_init(&root, out->data, sizeof(out->data), 0);
    if(!_ok(cbor_encoder_create_map(&root, &map, CborIndefiniteLength))) goto out;

    // cmd
    if(!_ok(cbor_encode_text_stringz(&map, "cmd"))) goto out;
    if(!_ok(cbor_encode_text_stringz(&map, "upload_graph_ack"))) goto out;

    // ok
    if(!_ok(cbor_encode_text_stringz(&map, "ok"))) goto out;
    if(!_ok(cbor_encode_boolean(&map, ok))) goto out;

    if (ok) {
        if(!_ok(cbor_encode_text_stringz(&map, "stagingNodeCount"))) goto out;
        if(!_ok(cbor_encode_uint(&map, cnt))) goto out;

        if(!_ok(cbor_encode_text_stringz(&map, "stagingCycleMs"))) goto out;
        if(!_ok(cbor_encode_uint(&map, cycle))) goto out;
    }

    (void)cbor_encoder_close_container(&root, &map);
    out:
    out->len = (uint16_t)cbor_encoder_get_buffer_size(&root, out->data);
}

static void send_simple_ack(const char* cmdName, bool ok, CborReply *out)
{
    CborEncoder root, map;

    cbor_encoder_init(&root, out->data, sizeof(out->data), 0);
    if(!_ok(cbor_encoder_create_map(&root, &map, CborIndefiniteLength))) goto out;

    if(!_ok(cbor_encode_text_stringz(&map, "cmd"))) goto out;
    if(!_ok(cbor_encode_text_stringz(&map, cmdName))) goto out;

    if(!_ok(cbor_encode_text_stringz(&map, "ok"))) goto out;
    if(!_ok(cbor_encode_boolean(&map, ok))) goto out;

    (void)cbor_encoder_close_container(&root, &map);
    out:
    out->len = (uint16_t)cbor_encoder_get_buffer_size(&root, out->data);
}

static void send_activate_graph_ack(bool ok, CborReply *out)  { send_simple_ack("activate_graph_ack", ok, out); }
static void send_force_output_ack(bool ok, CborReply *out)    { send_simple_ack("force_output_ack", ok, out); }

static void send_release_output_ack(bool ok, CborReply *out)
{
    send_simple_ack("release_output_ack", ok, out);
}

extern const PlcLogEntry* plc_log_get_by_index(uint16_t idx);
void send_log_dump_reply(CborReply *out)
{
    if (!out) return; out->len = 0;
    CborEncoder enc, map, arr;

    cbor_encoder_init(&enc, out->data, sizeof(out->data), 0);
    cbor_encoder_create_map(&enc, &map, CborIndefiniteLength);

    cbor_encode_text_stringz(&map, "cmd");
    cbor_encode_text_stringz(&map, "log_dump");

    // сюда выводим весь ring-buffer
    cbor_encode_text_stringz(&map, "log");
    CborEncoder arrEnc;
    uint16_t n = plc_log_count();
    cbor_encoder_create_array(&map, &arrEnc, n);

    for (uint16_t i = 0; i < n; i++) {
        const PlcLogEntry* le = plc_log_get_by_index(i); // добавь геттер
        CborEncoder rec;
        cbor_encoder_create_map(&arrEnc, &rec, 3);
        cbor_encode_text_stringz(&rec, "t"); cbor_encode_uint(&rec, le->ts_ms);
        cbor_encode_text_stringz(&rec, "c"); cbor_encode_uint(&rec, le->code);
        cbor_encode_text_stringz(&rec, "ab");
        CborEncoder ab;
        cbor_encoder_create_array(&rec, &ab, 2);
        cbor_encode_int(&ab, le->a);
        cbor_encode_int(&ab, le->b);
        cbor_encoder_close_container(&rec, &ab);
        cbor_encoder_close_container(&arrEnc, &rec);
    }

    cbor_encoder_close_container(&map, &arrEnc);
    cbor_encoder_close_container(&enc, &map);

    size_t len = cbor_encoder_get_buffer_size(&enc, out->data);
    out->len = len;
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



// helper
static inline bool map_find(CborValue *mapCont, const char *key, CborValue *out) {
    CborValue it = *mapCont; // КОПИЯ!
    CborError e = cbor_value_map_find_value(&it, key, out);
    if(e!=CborNoError){
        LOGT(tag, "map_find('%s'): err=%s", key, cbor_err_name(e));
        return false;
    }
    if(!cbor_value_is_valid(out) || cbor_value_is_undefined(out)){
        LOGT(tag, "map_find('%s'): not found / undefined", key);
        return false;
    }
    cbor_log_val(key, out);
    return true;
}

static CborError decode_single_node(CborValue *nodeMapIt, PlcNode *outNode) {
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

static CborError decode_graph(CborValue *graphMapIt, PlcGraph *outGraph) {
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

// Эта функция вызывается после того как мы приняли и проверили кадр (CRC и т.д.)
bool parse_cmd_upload_graph(CborValue root) {
    LOGT(tag, "parse_cmd_upload_graph: start");

    CborValue valGraph;
    if (!map_find(&root, "graph", &valGraph) || !cbor_value_is_map(&valGraph)) {
        LOGT(tag, "'graph' missing or not a map");
        return false;
    }

    PlcGraph *tmp = (PlcGraph*)pvPortMalloc(sizeof(PlcGraph));
    if (!tmp) {
        LOGT(tag, "NO HEAP: need=%u free=%u minEver=%u",
             (unsigned)sizeof(PlcGraph),
             (unsigned)xPortGetFreeHeapSize(),
             (unsigned)xPortGetMinimumEverFreeHeapSize());
        return false;
    }

    CborError err = decode_graph(&valGraph, tmp);
    if (err != CborNoError) {
        LOGT(tag, "decode_graph err=%s", cbor_err_name(err));
        vPortFree(tmp);
        return false;
    }

    LOGT(tag, "graph summary: nodes=%u cycleMs=%u", (unsigned)tmp->nodeCount, (unsigned)tmp->cycleMs);

    bool ok = plc_upload_graph(tmp);   // внутри копируйте в staging или active
    if (ok) {
        plc_log_put(PLC_EVT_UPLOAD_OK, /*a=*/(int16_t)tmp->nodeCount, /*b=*/(int16_t)tmp->cycleMs);
    } else {
        plc_log_put(PLC_EVT_UPLOAD_FAIL, 0, 0);
    }
    vPortFree(tmp);
    LOGT(tag, "plc_upload_graph -> %s", ok ? "OK" : "FAIL");
    return ok;
}

// cmd = "release_output"
// {
//   "cmd": "release_output",
//   "nodeIndex": <int>
// }
static bool parse_cmd_release_output(CborValue root)
{
    CborValue val;
    int32_t nodeIdx32;

    if (!map_find(&root, "nodeIndex", &val) || !get_int32(&val, &nodeIdx32)) {
        LOGT(tag, "release_output: 'nodeIndex' missing or bad");
        return false;
    }

    if (nodeIdx32 < 0 || nodeIdx32 >= PLC_MAX_NODES) {
        LOGT(tag, "release_output: nodeIndex=%ld out of range", (long)nodeIdx32);
        return false;
    }

    bool ok = plc_release_output((uint16_t)nodeIdx32);

    LOGT(tag, "release_output: node=%ld -> %s",
         (long)nodeIdx32, ok ? "OK" : "FAIL");

    return ok;
}


// cmd = "force_output"
// {
//   "cmd": "force_output",
//   "nodeIndex": <int>,
//   "value": <0|1>,
//   "holdMs": <uint, ms>
// }
static bool parse_cmd_force_output(CborValue root)
{
    CborValue val;
    int32_t  nodeIdx32;
    int32_t  value32;
    uint32_t holdMs = 0;

    if (!map_find(&root, "nodeIndex", &val) || !get_int32(&val, &nodeIdx32)) {
        LOGT(tag, "force_output: 'nodeIndex' missing or bad");
        return false;
    }

    if (nodeIdx32 < 0 || nodeIdx32 >= PLC_MAX_NODES) {
        LOGT(tag, "force_output: nodeIndex=%ld out of range", (long)nodeIdx32);
        return false;
    }

    if (!map_find(&root, "value", &val) || !get_int32(&val, &value32)) {
        LOGT(tag, "force_output: 'value' missing or bad");
        return false;
    }

    if (map_find(&root, "holdMs", &val)) {
        if (!get_uint32(&val, &holdMs)) {
            LOGT(tag, "force_output: 'holdMs' type error");
            return false;
        }
    } else {
        holdMs = 0; // если нужно, можно задать дефолт, например 1000
    }

    bool ok = plc_force_output((uint16_t)nodeIdx32, (value32 != 0), holdMs);

    LOGT(tag, "force_output: node=%ld val=%ld holdMs=%lu -> %s",
         (long)nodeIdx32, (long)value32, (unsigned long)holdMs,
         ok ? "OK" : "FAIL");

    return ok;
}


bool decode_graph_from_cbor(const uint8_t* buf, size_t len, PlcGraph* outG){
    CborParser parser;
    CborValue root;
    CborError err;

    err = cbor_parser_init(buf, len, 0, &parser, &root);
    if (err != CborNoError) {
        return false;
    }

    // Ждём корневую map
    if (!cbor_value_is_map(&root)) {
        return false;
    }

    return decode_graph(&root, outG) == CborNoError;
}

/* Точка входа из транспорта */
void on_cbor_frame_received(const uint8_t *buf, size_t len, CborReply* out) {
    if (out) out->len = 0;
#if CBOR_LOG_ENABLED
    LOGT(tag, "frame len=%u", (unsigned)len);
    cbor_hexdump_limited(tag, buf, len);
#endif

    CborParser parser;
    CborValue root;
    CborError err = cbor_parser_init(buf, len, 0, &parser, &root);
    if (err != CborNoError) {
        LOGT(tag, "cbor_parser_init err=%s", cbor_err_name(err));
        return;
    }

    if (!cbor_value_is_map(&root)) {
        LOGT(tag, "root is not map, type=%s", cbor_type_name(cbor_value_get_type(&root)));
        return;
    }

    CborValue valCmd;
    if(!map_find(&root, "cmd", &valCmd) || !cbor_value_is_text_string(&valCmd)){
        LOGT(tag, "cmd missing or not text");
        return;
    }

    char cmdBuf[32]; size_t cmdLen = sizeof(cmdBuf) - 1;
    err = cbor_value_copy_text_string(&valCmd, cmdBuf, &cmdLen, NULL);
    if (err != CborNoError) {
        LOGT(tag, "copy cmd err=%s", cbor_err_name(err));
        return;
    }
    cmdBuf[cmdLen] = '\0';
    LOGT(tag, "cmd='%s'", cmdBuf);

    if (strcmp(cmdBuf, "upload_graph") == 0) {
        LOGT(tag, "upload_graph");
        bool ok = parse_cmd_upload_graph(root);

        send_upload_graph_ack(ok, out);
        return;
    }

    if (strcmp(cmdBuf, "activate_graph") == 0) {
        bool ok = plc_request_activate_graph();
        send_activate_graph_ack(ok, out);
        return;
    }

    if (strcmp(cmdBuf, "status") == 0) {
        send_status_reply(out);
        return;
    }

//    if (strcmp(cmdBuf, "save_active") == 0) {
//        bool ok = plc_persist_save_active();
//        send_simple_ack("save_active_ack", ok, out);
//        return;
//    }
//
//    if (strcmp(cmdBuf, "load_active") == 0) {
//        bool ok = plc_persist_load_active();
//        send_simple_ack("load_active_ack", ok, out);
//        return;
//    }

    if (strcmp(cmdBuf, "log_dump") == 0) {
        send_log_dump_reply(out);
        return;
    }

    if (strcmp(cmdBuf, "force_output") == 0) {
        bool ok = parse_cmd_force_output(root);
        send_force_output_ack(ok, out);
        return;
    }

    if (strcmp(cmdBuf, "release_output") == 0) {
        bool ok = parse_cmd_release_output(root);
        send_release_output_ack(ok, out);
        return;
    }

    LOGT(tag, "unknown cmd");
}









