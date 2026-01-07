#include "plc.h"
#include "logf.h"
#include <string.h> // memcpy
#include "plc_log.h"

// ===== ЛОГИ PLC =====
#ifndef PLC_LOG_TAG
#define PLC_LOG_TAG "PLC"
#endif

#ifndef PLC_LOG_ENABLED
#define PLC_LOG_ENABLED 0
#endif

// Общие флаги
#ifndef PLC_LOG_VALIDATE
#define PLC_LOG_VALIDATE 1   // plc_validate_graph() — уже логирует WARN/ERR, оставим флаг для единообразия
#endif

#ifndef PLC_LOG_UPLOAD
#define PLC_LOG_UPLOAD 1     // plc_upload_graph() — факт, размеры, цикл
#endif

#ifndef PLC_LOG_ACTIVATE
#define PLC_LOG_ACTIVATE 1   // запрос и факт свапа активного графа
#endif

#ifndef PLC_LOG_TICK
#define PLC_LOG_TICK 0       // хартбит plc_tick() — раз в PLC_TICK_HEARTBEAT_MS
#endif
#ifndef PLC_TICK_HEARTBEAT_MS
#define PLC_TICK_HEARTBEAT_MS 1000u
#endif
#ifndef PLC_TICK_DT_WARN_MS
#define PLC_TICK_DT_WARN_MS  (PLC_TICK_HEARTBEAT_MS * 2u)   // предупреждение о больших dt
#endif

#ifndef PLC_LOG_INPUTS
#define PLC_LOG_INPUTS 1     // plc_refresh_inputs_hw(): DI/AI
#endif
#ifndef PLC_LOG_DI
#define PLC_LOG_DI 0
#endif
#ifndef PLC_LOG_AI
#define PLC_LOG_AI 0
#endif
#ifndef PLC_LOG_AI_DELTA
#define PLC_LOG_AI_DELTA 16  // сырой ADC: лог при изменении >= 16 ед.
#endif
#ifndef PLC_LOG_AI_PERIOD_MS
#define PLC_LOG_AI_PERIOD_MS 1000u // но не реже 1/сек на канал
#endif

#ifndef PLC_LOG_CMDS
#define PLC_LOG_CMDS 1       // plc_apply_pending_cmds(): принятые команды, завершение force
#endif

#ifndef PLC_LOG_GRAPH
#define PLC_LOG_GRAPH 1      // plc_graph_step(): события узлов (по изменениям)
#endif
#ifndef PLC_PWM_DELTA
#define PLC_PWM_DELTA 0      // PWM: лог при изменении duty >= 1%
#endif

// ---- локальные состояния для подавления лишних логов
static uint8_t s_di_init[PLC_MAX_NODES];
static uint8_t s_di_last[PLC_MAX_NODES];

static uint8_t s_ai_init[PLC_MAX_NODES];
static int32_t s_ai_last[PLC_MAX_NODES];
static uint32_t s_ai_last_ts[PLC_MAX_NODES];

static uint8_t s_do_init[PLC_MAX_NODES];
static uint8_t s_do_last[PLC_MAX_NODES];

static uint8_t s_pwm_init[PLC_MAX_NODES];
static int32_t s_pwm_last[PLC_MAX_NODES];

// --- Глобальная память ПЛК ---
// Можно сделать static, а доступ наружу через функции ниже
static bool    s_memBool[PLC_MEM_BOOL_COUNT];
static int32_t s_memInt[PLC_MEM_INT_COUNT];
static float   s_memReal[PLC_MEM_REAL_COUNT];

// Хартбит plc_tick()
static uint32_t s_tick_last_hb = 0;

// Утилиты
static inline int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }


PlcGraph g_activeGraph;
PlcGraph g_stagingGraph;

bool g_activeGraphValid = false;
bool g_stagingGraphValid = false;
volatile bool g_needSwapGraph = false;

//QueueHandle_t plcCmdQueue = NULL;

typedef struct {
    bool stable;         // стабилизированное состояние, которое отдаём наружу
    bool candidate;      // кандидат в новое состояние (последний сырый)
    uint32_t sinceMs;        // с какого времени держится candidate
    uint32_t debounceMs;     // порог стабилизации
    bool inited;
} DiFilterState;

#define PLC_HW_DI_COUNT 8
static DiFilterState di_filter[PLC_HW_DI_COUNT]; //DI 8шт
static const uint32_t DI_DEBOUNCE_DEFAULT_MS = 30; // по умолчанию 30 мс

/* ================== Внутренние хелперы ================== */

void plc_node_reset_runtime(PlcNode *n) {
    n->out.b = false;
    n->out.i = 0;
    n->out.f = 0.0f;

    n->sr_q = false;
    n->trig_prev = false;

    n->ton_active = false;
    n->ton_accum_ms = 0;

    n->toff_holding = false;
    n->toff_left_ms = 0;

    n->force_en = false;
    n->force_bool = false;
    n->force_left_ms = 0;

    // --- PID runtime ---
    n->pid_i = 0.0f;
    n->pid_prev_meas = 0.0f;
    n->pid_inited = false;

    // --- Filter / Ramp runtime ---
    n->filt_prev = 0.0f;
    n->filt_inited = false;

    n->ramp_prev = 0.0f;
    n->ramp_inited = false;

    n->log_accum_ms = 0;

    // CTU/CTD/CTUD
    n->acc = 0;
    n->prev_clk = false;

    n->pwm_zero_hold = false;
}

void plc_graph_reset_runtime(PlcGraph *g) {
    for (uint16_t i = 0; i < g->nodeCount; i++) {
        plc_node_reset_runtime(&g->nodes[i]);
    }
}

// --- validation helpers (PLC-style) ---
static inline PlcValueType node_type(const PlcGraph* g, int16_t idx) {
    if (!g) return PLC_VAL_FLOAT;
    if (idx < 0) return PLC_VAL_FLOAT;
    if ((uint16_t)idx >= g->nodeCount) return PLC_VAL_FLOAT;
    return g->nodes[idx].valueType;
}

static bool is_bool_node32(const PlcGraph *g, int idx) {
    if (idx < 0 || idx >= (int)g->nodeCount) return false;
    return g->nodes[idx].valueType == PLC_VAL_BOOL;
}

static inline bool is_bool_node16(const PlcGraph* g, int16_t idx) {
    return node_type(g, idx) == PLC_VAL_BOOL;
}

static inline bool is_numeric_node(const PlcGraph* g, int16_t idx) {
    PlcValueType t = node_type(g, idx);
    return (t == PLC_VAL_INT) || (t == PLC_VAL_FLOAT);
}

// FLOAT и INT считаем совместимыми (как в PLC: INT может быть приведён к REAL)
static inline bool types_compatible_nodes(const PlcGraph* g, int16_t a, int16_t b) {
    PlcValueType ta = node_type(g, a);
    PlcValueType tb = node_type(g, b);

    if (ta == tb) return true;

    const bool na = (ta == PLC_VAL_INT) || (ta == PLC_VAL_FLOAT);
    const bool nb = (tb == PLC_VAL_INT) || (tb == PLC_VAL_FLOAT);
    if (na && nb) return true;

    return false;
}

bool plc_validate_graph(PlcGraph *g) {
    if (g->nodeCount == 0) {
        LOGT("PLC", "g->nodeCount == 0. Return false");
        return false;
    }
    if (g->nodeCount > PLC_MAX_NODES) {
        LOGT("PLC", "g->nodeCount > PLC_MAX_NODES. Return false");
        return false;
    }

    // Нормализация периода цикла
    if (g->cycleMs < PLC_CYCLE_MS_MIN) g->cycleMs = PLC_CYCLE_MS_MIN;
    if (g->cycleMs > PLC_CYCLE_MS_MAX) g->cycleMs = PLC_CYCLE_MS_MAX;

    for (uint16_t i = 0; i < g->nodeCount; i++) {
        PlcNode *n = &g->nodes[i];

        // --- 1. Проверка id ---
        if (n->id != i) {
            LOGT("PLC", "id mismatch: node[%u].id=%d != %u", i, (int) n->id, i);
            return false;
        }

        // --- 2. Проверка типа ---
        if (n->type > PLC_NODE_MAX) {
            LOGT("PLC", "node[%u] type=%u > PLC_NODE_MAX", i, (unsigned) n->type);
            return false;
        }

        // --- 3. Ацикличность: inA/inB должны ссылаться только на узлы с меньшим индексом ---
        if (n->inA >= 0 && (uint16_t) n->inA >= i) {
            LOGT("PLC", "node[%u] inA=%d >= %u", i, (int) n->inA, i);
            return false;
        }
        if (n->inB >= 0 && (uint16_t) n->inB >= i) {
            LOGT("PLC", "node[%u] inB=%d >= %u", i, (int) n->inB, i);
            return false;
        }


        // --- 4. Проверка входов в зависимости от типа ---
        switch (n->type) {

            // Источники — не должны иметь входов
            case PLC_NODE_CONST_FLOAT:
            case PLC_NODE_CONST_BOOL:
            case PLC_NODE_CONST_INT:
            case PLC_NODE_DIGITAL_IN:
            case PLC_NODE_AI_IN:
                if (n->inA != -1 || n->inB != -1) {
                    LOGT("PLC", "node[%u] source type=%u must have inA=inB=-1", i, (unsigned) n->type);
                    return false;
                }
                break;

                // Узлы, требующие один вход
            case PLC_NODE_NOT:
            case PLC_NODE_DIGITAL_OUT:
            case PLC_NODE_TON:
            case PLC_NODE_TOFF:
            case PLC_NODE_R_TRIG:
            case PLC_NODE_F_TRIG:
            case PLC_NODE_PWM_OUT:
            case PLC_NODE_AO:
                if (n->inA < 0) {
                    LOGT("PLC", "node[%u] type=%u requires inA>=0", i, (unsigned) n->type);
                    return false;
                }
                break;

                // Узлы, требующие два входа
            case PLC_NODE_AND2:
            case PLC_NODE_OR2:
                if (n->inA < 0 || n->inB < 0) {
                    LOGT("PLC", "node[%u] AND/OR requires inA and inB", i);
                    return false;
                }
                if (!is_bool_node16(g, n->inA) || !is_bool_node16(g, n->inB)) {
                    LOGT("PLC", "node[%u] AND/OR requires BOOL inputs", i);
                    return false;
                }
                break;

            case PLC_NODE_COMPARE_GE:
            case PLC_NODE_COMPARE_GT:
            case PLC_NODE_COMPARE_LT:
                if (n->inA < 0 || n->inB < 0) {
                    LOGT("PLC", "node[%u] COMPARE requires inA and inB", i);
                    return false;
                }
                if (!types_compatible_nodes(g, n->inA, n->inB)) {
                    LOGT("PLC", "node[%u] COMPARE incompatible input types", i);
                    return false;
                }
                break;

                // SR latch: хотя бы один вход должен быть
            case PLC_NODE_SR:
                if (n->inA < 0 && n->inB < 0) {
                    LOGT("PLC", "node[%u] SR requires at least one input", i);
                    return false;
                }
                break;

                // MUX2: оба входа данных и селектор в paramInt
            case PLC_NODE_MUX2:
                if (n->inA < 0 || n->inB < 0) {
                    LOGT("PLC", "node[%u] MUX2 requires inA>=0 and inB>=0", i);
                    return false;
                }
                if (n->paramInt < 0 || n->paramInt >= (int32_t)i) {
                    LOGT("PLC", "node[%u] MUX2 selector paramInt=%d invalid (< %u)", i, (int) n->paramInt, i);
                    return false;
                }
                if (!is_bool_node32(g, n->paramInt)) {
                    LOGT("PLC", "node[%u] MUX2 selector must be BOOL (sel=%d)", i, (int)n->paramInt);
                    return false;
                }
                break;
            case PLC_NODE_TP:
                // Требует один вход (inA)
                if (n->inA < 0) {
                    LOGT("PLC", "node[%u] TP requires inA>=0", i);
                    return false;
                }
                if (n->paramMs > 600000u) { // 10 мин
                    LOGT("PLC", "node[%u] TP paramMs too large", i);
                    return false;
                }
                break;

            case PLC_NODE_HYST:
                // Один вход (float). Порог и ширина из параметров.
                if (n->inA < 0) {
                    LOGT("PLC", "node[%u] HYST requires inA>=0", i);
                    return false;
                }
                // paramInt может быть отрицат. не делаем жёсткой проверки: возьмём |h|
                break;

            case PLC_NODE_SCALE:
                if (n->inA < 0) {
                    LOGT("PLC", "node[%u] SCALE requires inA>=0", i);
                    return false;
                }
                break;

            case PLC_NODE_ADD:
                if (n->inA < 0 || n->inB < 0) {
                    LOGT("PLC", "node[%u] ADD requires inA>=0 and inB>=0", i);
                    return false;
                }
                break;

            case PLC_NODE_LIMIT:
                if (n->inA < 0) {
                    LOGT("PLC", "node[%u] LIMIT requires inA>=0", i);
                    return false;
                }
                // min/max в тысячных вмещаются в int/uint, без доп. проверок
                break;
            case PLC_NODE_PID:
                // Требуются оба входа: inA = SP, inB = PV
                if (n->inA < 0 || n->inB < 0) {
                    LOGT("PLC", "node[%u] PID requires inA(SP) and inB(PV)", i);
                    return false;
                }
                // Ki и Kd задаются как тысячные в paramInt/paramMs — допускаем 0 и выше
                if (n->paramInt < 0 || n->paramMs < 0) {
                    LOGT("PLC", "node[%u] PID Ki/Kd must be >= 0 (paramInt>=0, paramMs>=0)", i);
                    return false;
                }
                break;

            case PLC_NODE_ANALOG_AVG:
                if (n->inA < 0) {
                    LOGT("PLC", "node[%u] ANALOG_AVG requires inA>=0", i);
                    return false;
                }
                // 0 <= alpha <= 1 → paramInt 0..1000
                if (n->paramInt < 0 || n->paramInt > 1000) {
                    LOGT("PLC", "node[%u] ANALOG_AVG alpha (paramInt) must be 0..1000", i);
                    return false;
                }
                break;

            case PLC_NODE_RAMP:
                if (n->inA < 0) {
                    LOGT("PLC", "node[%u] RAMP requires inA>=0", i);
                    return false;
                }
                // rate >= 0 (единиц в секунду), задаётся в тысячных
                if (n->paramInt < 0) {
                    LOGT("PLC", "node[%u] RAMP rate (paramInt) must be >=0", i);
                    return false;
                }
                break;
            case PLC_NODE_LOG:
                // Должен быть вход
                if (n->inA < 0) {
                    LOGT("PLC", "node[%u] LOG requires inA>=0", i);
                    return false;
                }
                // paramMs = период логирования, мс (минимум, скажем, 10 мс)
                if (n->paramMs < 10u) {
                    LOGT("PLC", "node[%u] LOG paramMs must be >=10 ms", i);
                    return false;
                }
                break;
            case PLC_NODE_CTD:
            case PLC_NODE_CTU:
                // inA — обязательный CLK
                if (n->inA < 0) {
                    LOGT("PLC", "node[%u] CTU requires inA>=0 (CLK)", i);
                    return false;
                }
                // inB — опциональный RST, может быть -1
                // paramInt — PV, допускаем 0 и больше
                if (n->paramInt < 0) {
                    LOGT("PLC", "node[%u] CTU paramInt(PV) must be >=0", i);
                    return false;
                }
                // valueType: либо INT, либо BOOL
                if (n->valueType != PLC_VAL_INT && n->valueType != PLC_VAL_BOOL) {
                    LOGT("PLC", "node[%u] CTU valueType must be INT or BOOL", i);
                    return false;
                }
                break;
            case PLC_NODE_CTUD:
                // Должен быть хотя бы один вход: CU (inA) или CD (inB)
                if (n->inA < 0 && n->inB < 0) {
                    LOGT("PLC", "node[%u] CTUD requires inA>=0 or inB>=0", i);
                    return false;
                }
                // Для CTUD paramInt сейчас не используется, но проверим на >=0 для совместимости
                if (n->paramInt < 0) {
                    LOGT("PLC", "node[%u] CTUD paramInt must be >=0", i);
                    return false;
                }
                break;
            case PLC_NODE_WINDOW_CHECK:
                // Требуется вход A (значение x)
                if (n->inA < 0) {
                    LOGT("PLC", "node[%u] WINDOW_CHECK requires inA>=0", i);
                    return false;
                }
                // Ширина окна в тысячных, допускаем 0 и выше
                if (n->paramInt < 0) {
                    LOGT("PLC", "node[%u] WINDOW_CHECK paramInt(width*1000) must be >=0", i);
                    return false;
                }
                // Выход, как правило, BOOL
                if (n->valueType != PLC_VAL_BOOL) {
                    LOGT("PLC", "node[%u] WINDOW_CHECK normally outputs BOOL, but valueType=%u",
                         i, (unsigned)n->valueType);
                }
                break;

            case PLC_NODE_SAFE_OUTPUT:
                // Управляющий сигнал обязателен
                if (n->inA < 0) {
                    LOGT("PLC", "node[%u] SAFE_OUTPUT requires inA>=0 (normal value)", i);
                    return false;
                }
                // inB (ALLOW) может быть -1 (тогда всегда разрешено)
                // Тут без дополнительных проверок по param*
                break;

            case PLC_NODE_ALARM_GEN:
                // Требуется условие (bool) на входе A
                if (n->inA < 0) {
                    LOGT("PLC", "node[%u] ALARM_GEN requires inA>=0 (condition)", i);
                    return false;
                }
                if (n->valueType != PLC_VAL_BOOL) {
                    LOGT("PLC", "node[%u] ALARM_GEN normally outputs BOOL, but valueType=%u",
                         i, (unsigned)n->valueType);
                }
                break;

            case PLC_NODE_ALARM_LATCH:
                // inA — условие аварии, inB — ACK/RESET
                if (n->inA < 0) {
                    LOGT("PLC", "node[%u] ALARM_LATCH requires inA>=0 (condition)", i);
                    return false;
                }
                // inB может быть -1, тогда квитирование не предусмотрено
                if (n->valueType != PLC_VAL_BOOL) {
                    LOGT("PLC", "node[%u] ALARM_LATCH normally outputs BOOL, but valueType=%u",
                         i, (unsigned)n->valueType);
                }
                break;

            case PLC_NODE_HEARTBEAT:
                // Источник самогенерируемый — входов не должно быть
                if (n->inA != -1 || n->inB != -1) {
                    LOGT("PLC", "node[%u] HEARTBEAT must have inA=inB=-1", i);
                    return false;
                }
                if (n->paramMs == 0u) {
                    LOGT("PLC", "node[%u] HEARTBEAT paramMs must be >0", i);
                    return false;
                }
                if (n->valueType != PLC_VAL_BOOL) {
                    LOGT("PLC", "node[%u] HEARTBEAT normally outputs BOOL, but valueType=%u",
                         i, (unsigned)n->valueType);
                }
                break;
            case PLC_NODE_MEM_BOOL:
                // Можно без входов (чистое чтение), либо с inA (запись) и опциональным inB (WE)
                if (n->paramInt < 0 || n->paramInt >= PLC_MEM_BOOL_COUNT) {
                    LOGT("PLC", "node[%u] MEM_BOOL paramInt(index) out of range: %d",
                         i, (int)n->paramInt);
                    return false;
                }
                if (n->valueType != PLC_VAL_BOOL) {
                    LOGT("PLC", "node[%u] MEM_BOOL must have valueType=BOOL, got=%u",
                         i, (unsigned)n->valueType);
                    return false;
                }
                break;

            case PLC_NODE_MEM_INT:
                if (n->paramInt < 0 || n->paramInt >= PLC_MEM_INT_COUNT) {
                    LOGT("PLC", "node[%u] MEM_INT paramInt(index) out of range: %d",
                         i, (int)n->paramInt);
                    return false;
                }
                if (n->valueType != PLC_VAL_INT) {
                    LOGT("PLC", "node[%u] MEM_INT must have valueType=INT, got=%u",
                         i, (unsigned)n->valueType);
                    return false;
                }
                break;

            case PLC_NODE_MEM_REAL:
                if (n->paramInt < 0 || n->paramInt >= PLC_MEM_REAL_COUNT) {
                    LOGT("PLC", "node[%u] MEM_REAL paramInt(index) out of range: %d",
                         i, (int)n->paramInt);
                    return false;
                }
                if (n->valueType != PLC_VAL_FLOAT) {
                    LOGT("PLC", "node[%u] MEM_REAL must have valueType=FLOAT, got=%u",
                         i, (unsigned)n->valueType);
                    return false;
                }
                break;
            case PLC_NODE_FILTER_AVG:
                // Нужен вход A (аналоговый сигнал)
                if (n->inA < 0) {
                    LOGT("PLC", "node[%u] FILTER_AVG requires inA>=0", i);
                    return false;
                }
                // alpha в тысячных, 0..1000 (0..1.0)
                if (n->paramInt < 0 || n->paramInt > 1000) {
                    LOGT("PLC", "node[%u] FILTER_AVG paramInt(alpha*1000) must be in [0..1000], got=%d",
                         i, (int)n->paramInt);
                    return false;
                }
                if (n->valueType != PLC_VAL_FLOAT) {
                    LOGT("PLC", "node[%u] FILTER_AVG must have valueType=FLOAT, got=%u",
                         i, (unsigned)n->valueType);
                    return false;
                }
                break;

            case PLC_NODE_MATH_OP:
                // Два входа обязательны: A и B
                if (n->inA < 0 || n->inB < 0) {
                    LOGT("PLC", "node[%u] MATH_OP requires inA>=0 and inB>=0", i);
                    return false;
                }
                // paramInt = код операции: 0=ADD,1=SUB,2=MUL,3=DIV
                if (n->paramInt < 0 || n->paramInt > 3) {
                    LOGT("PLC", "node[%u] MATH_OP paramInt(op) must be in [0..3], got=%d",
                         i, (int)n->paramInt);
                    return false;
                }
                // Поддерживаем INT и FLOAT
                if (n->valueType != PLC_VAL_INT && n->valueType != PLC_VAL_FLOAT) {
                    LOGT("PLC", "node[%u] MATH_OP must have valueType=INT or FLOAT, got=%u",
                         i, (unsigned)n->valueType);
                    return false;
                }
                break;

            default:
                break;
        }

        // --- 5. Проверка параметров таймеров ---
        if ((n->type == PLC_NODE_TON || n->type == PLC_NODE_TOFF) &&
            n->paramMs > 600000u) { // 10 мин
            LOGT("PLC", "node[%u] timer paramMs=%u > 600000", i, (unsigned) n->paramMs);
            return false;
        }

        // --- 6. Проверка valueType (только предупреждения) ---
        if ((n->type == PLC_NODE_DIGITAL_IN || n->type == PLC_NODE_AND2 ||
             n->type == PLC_NODE_OR2 || n->type == PLC_NODE_NOT ||
             n->type == PLC_NODE_SR || n->type == PLC_NODE_TON ||
             n->type == PLC_NODE_TOFF || n->type == PLC_NODE_R_TRIG ||
             n->type == PLC_NODE_F_TRIG || n->type == PLC_NODE_DIGITAL_OUT) &&
            n->valueType != PLC_VAL_BOOL) {
            LOGT("PLC", "node[%u] type=%u normally outputs BOOL, but valueType=%u", i, (unsigned) n->type,
                 (unsigned) n->valueType);
        }

        if ((n->type == PLC_NODE_CONST_INT) && n->valueType != PLC_VAL_INT) {
            LOGT("PLC", "node[%u] CONST_INT should have valueType=INT, now=%u", i, (unsigned) n->valueType);
        }

        if ((n->type == PLC_NODE_AI_IN || n->type == PLC_NODE_PWM_OUT || n->type == PLC_NODE_MUX2) &&
            n->valueType != PLC_VAL_FLOAT) {
            LOGT("PLC", "node[%u] type=%u normally outputs FLOAT, but valueType=%u", i, (unsigned) n->type,
                 (unsigned) n->valueType);
        }

        if (n->type == PLC_NODE_TP || n->type == PLC_NODE_HYST) {
            if (n->valueType != PLC_VAL_BOOL) {
                LOGT("PLC", "node[%u] type=%u normally outputs BOOL, but valueType=%u", i, (unsigned) n->type,
                     (unsigned) n->valueType);
            }
        }
        if (n->type == PLC_NODE_SCALE || n->type == PLC_NODE_ADD || n->type == PLC_NODE_LIMIT) {
            if (n->valueType != PLC_VAL_FLOAT) {
                LOGT("PLC", "node[%u] type=%u normally outputs FLOAT, but valueType=%u", i, (unsigned) n->type,
                     (unsigned) n->valueType);
            }
        }

        if (n->type == PLC_NODE_PID && n->valueType != PLC_VAL_FLOAT) {
            LOGT("PLC", "node[%u] PID normally outputs FLOAT, but valueType=%u", i, (unsigned) n->valueType);
        }

        // --- 7. Проверка hw индексов ---
        if ((n->type == PLC_NODE_DIGITAL_IN || n->type == PLC_NODE_DIGITAL_OUT ||
             n->type == PLC_NODE_AI_IN || n->type == PLC_NODE_PWM_OUT) &&
            n->paramInt < 0) {
            LOGT("PLC", "node[%u] type=%u uses paramInt as HW index, but paramInt<0", i, (unsigned) n->type);
        }
    }

    return true;
}


/* ================== API для TaskComms ================== */

bool plc_upload_graph(const PlcGraph *src) {
#if PLC_LOG_ENABLED && PLC_LOG_UPLOAD
    LOGT(PLC_LOG_TAG, "upload: requested (nodeCount=%u, cycleMs=%u)", (unsigned) src->nodeCount,
         (unsigned) src->cycleMs);
#endif

    g_stagingGraph = *src;

    if (!plc_validate_graph(&g_stagingGraph)) {
#if PLC_LOG_ENABLED && PLC_LOG_UPLOAD
        LOGT(PLC_LOG_TAG, "upload: INVALID graph");
#endif
        g_stagingGraphValid = false;
        plc_log_put(PLC_EVT_VALIDATE_ERR, 0, 0);
        return false;
    }

#if PLC_LOG_ENABLED && PLC_LOG_UPLOAD
    LOGT(PLC_LOG_TAG, "upload: graph valid (nodes=%u, cycleMs=%u)", (unsigned) g_stagingGraph.nodeCount,
         (unsigned) g_stagingGraph.cycleMs);
#endif

    plc_graph_reset_runtime(&g_stagingGraph);
#if PLC_LOG_ENABLED && PLC_LOG_UPLOAD
    LOGT(PLC_LOG_TAG, "upload: runtime reset done");
#endif

    g_stagingGraphValid = true;
//  g_needSwapGraph = true; // оставлено как в твоём коде (закомментировано)
    return true;
}


bool plc_request_activate_graph(void) {
    if (!g_stagingGraphValid) {
#if PLC_LOG_ENABLED && PLC_LOG_ACTIVATE
        LOGT(PLC_LOG_TAG, "activate: staging invalid -> reject");
#endif
        return false;
    }
    g_needSwapGraph = true; // TaskPlcScan обработает
#if PLC_LOG_ENABLED && PLC_LOG_ACTIVATE
    LOGT(PLC_LOG_TAG, "activate: requested (will swap on next tick)");
#endif

    plc_log_put(PLC_EVT_ACTIVATE_OK, (int16_t)g_stagingGraph.nodeCount, (int16_t)g_stagingGraph.cycleMs);
    return true;
}

bool plc_release_output(uint16_t nodeIndex)
{
#if PLC_LOG_ENABLED && PLC_LOG_CMDS
    LOGT(PLC_LOG_TAG, "release_output: req node=%u", (unsigned)nodeIndex);
#endif

    if (!g_activeGraphValid) {
#if PLC_LOG_ENABLED && PLC_LOG_CMDS
        LOGT(PLC_LOG_TAG, "release_output: no active graph");
#endif
        return false;
    }

    if (nodeIndex >= g_activeGraph.nodeCount) {
#if PLC_LOG_ENABLED && PLC_LOG_CMDS
        LOGT(PLC_LOG_TAG, "release_output: bad nodeIndex=%u (nodes=%u)",
             (unsigned)nodeIndex, (unsigned)g_activeGraph.nodeCount);
#endif
        return false;
    }

    PlcNode *n = &g_activeGraph.nodes[nodeIndex];

    // Смысла нет снимать форс с не-выходных узлов, но и не ошибка
    if (n->type != PLC_NODE_DIGITAL_OUT &&
        n->type != PLC_NODE_PWM_OUT &&
        n->type != PLC_NODE_AO) {
#if PLC_LOG_ENABLED && PLC_LOG_CMDS
        LOGT(PLC_LOG_TAG, "release_output: node[%u] type=%u not DO/PWM",
             (unsigned)nodeIndex, (unsigned)n->type);
#endif
        // можно вернуть true, чтобы на верхнем уровне не считалось ошибкой
        return true;
    }


        n->force_en      = false;
        n->flags        &= ~PLC_NODE_FLAG_FORCE;
        n->force_left_ms = 0;
        // force_bool можно не трогать, он больше не используется


#if PLC_LOG_ENABLED && PLC_LOG_CMDS
    LOGT(PLC_LOG_TAG, "release_output: node[%u] OK", (unsigned)nodeIndex);
#endif

    return true;
}


bool plc_force_output(uint16_t nodeIndex, bool value, uint32_t holdMs)
{
#if PLC_LOG_ENABLED && PLC_LOG_CMDS
    LOGT(PLC_LOG_TAG, "force_output: req node=%u val=%u holdMs=%u",
         (unsigned)nodeIndex, (unsigned)value, (unsigned)holdMs);
#endif


    do {
        if (!g_activeGraphValid) {
#if PLC_LOG_ENABLED && PLC_LOG_CMDS
            LOGT(PLC_LOG_TAG, "force_output: no active graph");
#endif
            break;
        }

        if (nodeIndex >= g_activeGraph.nodeCount) {
#if PLC_LOG_ENABLED && PLC_LOG_CMDS
            LOGT(PLC_LOG_TAG, "force_output: bad nodeIndex=%u (nodeCount=%u)",
                 (unsigned)nodeIndex, (unsigned)g_activeGraph.nodeCount);
#endif
            break;
        }

        PlcNode *n = &g_activeGraph.nodes[nodeIndex];

        if (n->type != PLC_NODE_DIGITAL_OUT &&
            n->type != PLC_NODE_PWM_OUT &&
            n->type != PLC_NODE_AO) {
#if PLC_LOG_ENABLED && PLC_LOG_CMDS
            LOGT(PLC_LOG_TAG, "force_output: node[%u] type=%u not DO/PWM",
                 (unsigned)nodeIndex, (unsigned)n->type);
#endif
            break;
        }

        n->force_en      = true;
        n->force_bool    = value;
        n->force_left_ms = holdMs;
        n->flags        |= PLC_NODE_FLAG_FORCE;

#if PLC_LOG_ENABLED && PLC_LOG_CMDS
        LOGT(PLC_LOG_TAG, "force_output: node[%u] en=1 val=%u left=%u",
             (unsigned)nodeIndex, (unsigned)n->force_bool, (unsigned)n->force_left_ms);
#endif


        return true;
    } while (0);


    return false;
}



/* ================== Основной тик ПЛК (вызов из TaskPlcScan) ================== */

// ---- PLC runtime metrics ----
uint32_t g_scan_cnt = 0;
uint32_t g_scan_ms_accum = 0;     // сумма длительностей шага (мс)
uint32_t g_scan_ms_max = 0;       // максимум за период
uint32_t g_scan_long_steps = 0;   // шагов > g_activeGraph.cycleMs

#define SCAN_STAT_WINDOW 1000u

void plc_metrics_on_cycle(uint32_t execMs, uint32_t periodMs)
{
    g_scan_cnt++;
    g_scan_ms_accum += execMs;
    if (execMs > g_scan_ms_max) g_scan_ms_max = execMs;
    if (execMs > periodMs) g_scan_long_steps++;

    if (g_scan_cnt >= SCAN_STAT_WINDOW) {
        // можно сохранить lastAvg отдельно, а тут сбросить
        g_scan_cnt = 0;
        g_scan_ms_accum = 0;
        g_scan_ms_max = 0;
    }
}

static uint32_t s_lastNowMs = 0;

void plc_tick(uint32_t nowMs)
{
    uint32_t periodMs = 10;
    if (g_activeGraphValid && g_activeGraph.cycleMs > 0) {
        periodMs = g_activeGraph.cycleMs;
    }

    uint32_t dt;
    if (s_lastNowMs == 0) {
        dt = periodMs;                 // первый цикл
    } else {
        dt = nowMs - s_lastNowMs;      // реальное время
    }
    s_lastNowMs = nowMs;

    // clamp, чтобы не “взорвать” PID/таймеры после паузы
    // (например, после дебага/стопа/высоких приоритетов)
    const uint32_t DT_MAX = periodMs * 4u;   // типично 2..10 периодов
    if (dt > DT_MAX) dt = DT_MAX;
    if (dt == 0) dt = 1;                     // защита от 0

    // дальше все таймеры/пиды используют dt (а не periodMs)
    // plc_refresh_inputs_hw(nowMs, dt);
    // plc_eval_graph(dt);


    // 1) активация нового графа
    if (g_needSwapGraph && g_stagingGraphValid) {


            g_activeGraph = g_stagingGraph;
            g_activeGraphValid = true;
            g_needSwapGraph = false;
            plc_graph_reset_runtime(&g_activeGraph);
            memset(di_filter, 0, sizeof(di_filter));


        if (PLC_RESET_MEM_ON_ACTIVATE) {
            memset(s_memBool, 0, sizeof(s_memBool));
            memset(s_memInt,  0, sizeof(s_memInt));
            memset(s_memReal, 0, sizeof(s_memReal));
        }

        // СБРОС ВСЕХ ВЫХОДОВ
        plc_hw_reset_outputs();



#if PLC_LOG_ENABLED && PLC_LOG_ACTIVATE
        LOGT(PLC_LOG_TAG, "activate: swapped (nodes=%u, cycleMs=%u)",
             (unsigned) g_activeGraph.nodeCount, (unsigned) g_activeGraph.cycleMs);
#endif
    }

    // 2) нет активного графа
    if (!g_activeGraphValid) {
#if PLC_LOG_ENABLED && PLC_LOG_TICK
        uint32_t tnow = nowMs;
        if (tnow - s_tick_last_hb >= PLC_TICK_HEARTBEAT_MS) {
            s_tick_last_hb = tnow;
            LOGT(PLC_LOG_TAG, "tick: idle (no active graph)");
        }
#endif
        iwdg_feed();
        return;
    }

    // 3) dt мониторинг
#if PLC_LOG_ENABLED && PLC_LOG_TICK
    if (dt > PLC_TICK_DT_WARN_MS) {
        LOGT(PLC_LOG_TAG, "tick: dt=%u ms (WARN)", (unsigned) dt);
    } else {
        uint32_t tnow = nowMs;
        if (tnow - s_tick_last_hb >= PLC_TICK_HEARTBEAT_MS) {
            s_tick_last_hb = tnow;
            LOGT(PLC_LOG_TAG, "tick: ok dt=%u ms nodes=%u cycle=%u",
                 (unsigned) dt, (unsigned) g_activeGraph.nodeCount, (unsigned) g_activeGraph.cycleMs);
        }
    }
#endif

    // 4) цикл ПЛК
    plc_refresh_inputs_hw(nowMs, &g_activeGraph);
    plc_apply_pending_cmds(&g_activeGraph, dt);
    plc_graph_step(&g_activeGraph, dt);

    // 5) watchdog
    iwdg_feed();
}


/* ================== Реализация refresh_inputs/apply_cmds/graph_step ================== */

void plc_refresh_inputs_hw(uint32_t nowMs, PlcGraph *g) {
    for (uint16_t i = 0; i < g->nodeCount; i++) {
        PlcNode *n = &g->nodes[i];
        switch (n->type) {
            case PLC_NODE_DIGITAL_IN: {
                const uint32_t tnow = nowMs;

                int ch = n->paramInt;
                if (ch < 0 || ch >= (int)(sizeof(di_filter)/sizeof(di_filter[0]))) {
#if PLC_LOG_ENABLED
                    LOGW(PLC_LOG_TAG, "DI node %u invalid channel=%d", (unsigned)i, ch);
#endif
                    // Поведение по умолчанию: считаем вход = 0
                    n->out.b = false;
                    break;
                }

                DiFilterState *fs = &di_filter[ch];
                bool raw = hw_read_discrete_input(ch);

                if (!fs->inited) {
                    fs->inited = true;
                    fs->stable = raw;
                    fs->candidate = raw;
                    fs->sinceMs = tnow;
                    if (fs->debounceMs == 0) fs->debounceMs = DI_DEBOUNCE_DEFAULT_MS;
#if PLC_LOG_ENABLED && PLC_LOG_INPUTS && PLC_LOG_DI
                    LOGT(PLC_LOG_TAG, "DI[%u] init ch=%d stable=%u db=%u",
                         (unsigned) i, (int) n->paramInt, (unsigned) fs->stable, (unsigned) fs->debounceMs);
#endif
                }

                if (raw != fs->candidate) {
                    fs->candidate = raw;
                    fs->sinceMs = tnow;
                } else {
                    if ((tnow - fs->sinceMs) >= fs->debounceMs && fs->stable != fs->candidate) {
                        fs->stable = fs->candidate;
#if PLC_LOG_ENABLED && PLC_LOG_INPUTS && PLC_LOG_DI
                        if (!s_di_init[i] || s_di_last[i] != (uint8_t) fs->stable) {
                            s_di_init[i] = 1;
                            s_di_last[i] = (uint8_t) fs->stable;
                            LOGT(PLC_LOG_TAG, "DI[%u] ch=%d STABLE->%u", (unsigned) i, (int) n->paramInt,
                                 (unsigned) fs->stable);
                        }
#endif
                    }
                }

                n->out.b = fs->stable;
                break;
            }
            case PLC_NODE_AI_IN: {
                int32_t raw = hw_read_adc_input(n->paramInt); // 0..4095

                // Нормализованный сигнал 0..1 (PLC-стайл)
                float x = (float)raw * (1.0f / 4095.0f);

                // RAW оставим только если явно нужен
                if (n->paramMs > 0) {
                    n->out.i = raw;
                    n->out.f = (float) raw;
                } else {
                    n->out.f = x;  // default: FLOAT
                }

#if PLC_LOG_ENABLED && PLC_LOG_INPUTS && PLC_LOG_AI
                uint8_t do_log = 0;
                uint32_t tnow = nowMs;
                if (!s_ai_init[i]) {
                    s_ai_init[i] = 1;
                    s_ai_last[i] = raw;
                    s_ai_last_ts[i] = tnow;
                    do_log = 1;
                } else {
                    int32_t d = iabs32(raw - s_ai_last[i]);
                    if ((tnow - s_ai_last_ts[i]) >= PLC_LOG_AI_PERIOD_MS || d >= PLC_LOG_AI_DELTA) do_log = 1;
                }
                if (do_log) {
                    LOGT(PLC_LOG_TAG, "AI[%u] ch=%d raw=%ld d=%ld",
                         (unsigned) i, (int) n->paramInt, (long) raw, (long) (raw - s_ai_last[i]));
                    s_ai_last[i] = raw;
                    s_ai_last_ts[i] = tnow;
                }
#endif
                break;
            }
            default:
                break;
        }
    }
}

void plc_apply_pending_cmds(PlcGraph *g, uint32_t dt_ms)
{
    for (uint16_t i = 0; i < g->nodeCount; i++) {
        PlcNode *n = &g->nodes[i];

        if (n->force_en && (n->flags & PLC_NODE_FLAG_FORCE)) {

            // 0 = бесконечный форс, таймер не считаем вообще
            if (n->force_left_ms == 0) {
                continue;
            }

            if (n->force_left_ms <= dt_ms) {
                n->force_en      = false;
                n->flags        &= ~PLC_NODE_FLAG_FORCE;
                n->force_left_ms = 0;
#if PLC_LOG_ENABLED && PLC_LOG_CMDS
                LOGT(PLC_LOG_TAG, "force: node[%u] expired", (unsigned)i);
#endif
            } else {
                n->force_left_ms -= dt_ms;
            }
        }
    }
}

static inline float get_float(const PlcNode *n) {
    if (!n) return 0.0f;
    switch (n->valueType) {
        case PLC_VAL_FLOAT:
            return n->out.f;
        case PLC_VAL_INT:
            return (float) n->out.i;
        case PLC_VAL_BOOL:
            return n->out.b ? 1.0f : 0.0f;
        default:
            return 0.0f;
    }
}

static inline int32_t get_int(const PlcNode *n) {
    if (!n) return 0;
    switch (n->valueType) {
        case PLC_VAL_INT:
            return n->out.i;
        case PLC_VAL_FLOAT:
            return (int32_t) (n->out.f + (n->out.f >= 0 ? 0.5f : -0.5f)); // округление
        case PLC_VAL_BOOL:
            return n->out.b ? 1 : 0;
        default:
            return 0;
    }
}

static inline bool get_bool(const PlcNode *n) {
    if (!n) return false;
    switch (n->valueType) {
        case PLC_VAL_BOOL:
            return n->out.b;
        case PLC_VAL_INT:
            return n->out.i != 0;
        case PLC_VAL_FLOAT:
            return n->out.f != 0.0f;
        default:
            return false;
    }
}

void plc_graph_step(PlcGraph *g, uint32_t dt_ms) {
    for (uint16_t i = 0; i < g->nodeCount; i++) {
        PlcNode *self = &g->nodes[i];
        const PlcNode *a = (self->inA >= 0) ? &g->nodes[self->inA] : NULL;
        const PlcNode *b = (self->inB >= 0) ? &g->nodes[self->inB] : NULL;

        switch (self->type) {

            case PLC_NODE_CONST_BOOL:
                self->out.b = (self->paramInt != 0);
                break;

            case PLC_NODE_CONST_INT:   // лучше переименовать в PLC_NODE_CONST, но можно оставить id
            {
                switch (self->valueType) {
                    case PLC_VAL_BOOL:
                        // константа-логика: 0 -> false, всё остальное -> true
                        self->out.b = (self->paramInt != 0);
                        break;

                    case PLC_VAL_INT:
                        // как было
                        self->out.i = self->paramInt;
                        break;

                    case PLC_VAL_FLOAT:
                    default: {
                        float v = self->paramFloat;

                        // необязательно, но удобно:
                        // если paramFloat == 0, а paramInt != 0, считаем, что paramInt в тысячных
                        if (v == 0.0f && self->paramInt != 0) {
                            v = (float)self->paramInt * 0.001f;   // 2500 -> 2.5
                        }

                        self->out.f = v;
                        break;
                    }
                }
                break;
            }

            case PLC_NODE_DIGITAL_IN:
            case PLC_NODE_AI_IN:
                // уже обновлено в plc_refresh_inputs_hw
                break;

            case PLC_NODE_AND2:
                self->out.b = get_bool(a) && get_bool(b);
                break;

            case PLC_NODE_OR2:
                self->out.b = get_bool(a) || get_bool(b);
                break;

            case PLC_NODE_NOT:
                self->out.b = !get_bool(a);
                break;

            case PLC_NODE_SR: {
                bool setSig = get_bool(a);
                bool rstSig = get_bool(b);
                if (rstSig) {
                    self->sr_q = false;
                } else if (setSig) {
                    self->sr_q = true;
                }
                self->out.b = self->sr_q;
                break;
            }

            case PLC_NODE_TON: {
                bool inSig = get_bool(a);
                if (inSig) {
                    if (!self->ton_active) {
                        self->ton_active = true;
                        self->ton_accum_ms = 0;
#if PLC_LOG_ENABLED && PLC_LOG_GRAPH
                        LOGT(PLC_LOG_TAG, "TON[%u] start T=%u", (unsigned) i, (unsigned) self->paramMs);
#endif
                    } else {
                        if (self->ton_accum_ms < self->paramMs) {
                            self->ton_accum_ms += dt_ms;
                            if (self->ton_accum_ms > self->paramMs) self->ton_accum_ms = self->paramMs;
                        }
                    }
                } else {
                    if (self->ton_active) {
#if PLC_LOG_ENABLED && PLC_LOG_GRAPH
                        LOGT(PLC_LOG_TAG, "TON[%u] reset (acc=%u)", (unsigned) i, (unsigned) self->ton_accum_ms);
#endif
                    }
                    self->ton_active = false;
                    self->ton_accum_ms = 0;
                }
                bool prev = self->out.b;
                self->out.b = (self->ton_accum_ms >= self->paramMs);
#if PLC_LOG_ENABLED && PLC_LOG_GRAPH
                if (self->out.b != prev) LOGT(PLC_LOG_TAG, "TON[%u] OUT->%u", (unsigned) i, (unsigned) self->out.b);
#endif
                break;
            }


            case PLC_NODE_TOFF: {
                bool inSig = get_bool(a);
                if (inSig) {
                    self->out.b = true;
                    if (!self->toff_holding) {
                        self->toff_holding = true;
                        self->toff_left_ms = self->paramMs;
#if PLC_LOG_ENABLED && PLC_LOG_GRAPH
                        LOGT(PLC_LOG_TAG, "TOFF[%u] hold start T=%u", (unsigned) i, (unsigned) self->paramMs);
#endif
                    }
                } else {
                    if (self->toff_holding) {
                        if (self->toff_left_ms <= dt_ms) {
                            self->toff_left_ms = 0;
                            self->toff_holding = false;
#if PLC_LOG_ENABLED && PLC_LOG_GRAPH
                            LOGT(PLC_LOG_TAG, "TOFF[%u] hold end", (unsigned) i);
#endif
                        } else {
                            self->toff_left_ms -= dt_ms;
                        }
                    }
                    self->out.b = self->toff_holding;
                }
                break;
            }


            case PLC_NODE_R_TRIG: {
                bool cur = get_bool(a);
                bool pulse = (cur && !self->trig_prev);
                self->out.b = pulse;
                self->trig_prev = cur;
#if PLC_LOG_ENABLED && PLC_LOG_GRAPH
                if (pulse) LOGT(PLC_LOG_TAG, "R_TRIG[%u] PULSE", (unsigned) i);
#endif
                break;
            }
            case PLC_NODE_F_TRIG: {
                bool cur = get_bool(a);
                bool pulse = (!cur && self->trig_prev);
                self->out.b = pulse;
                self->trig_prev = cur;
#if PLC_LOG_ENABLED && PLC_LOG_GRAPH
                if (pulse) LOGT(PLC_LOG_TAG, "F_TRIG[%u] PULSE", (unsigned) i);
#endif
                break;
            }


            case PLC_NODE_COMPARE_LT: {
                float aa = a ? get_float(a) : 0.0f;
                float bb = b ? get_float(b) : 0.0f;
                self->out.b = (aa < bb);
                break;
            }

            case PLC_NODE_COMPARE_GT: {
                float aa = a ? get_float(a) : 0.0f;
                float bb = b ? get_float(b) : 0.0f;
                self->out.b = (aa > bb);
                break;
            }

            case PLC_NODE_COMPARE_GE: {
                float aa = a ? get_float(a) : 0.0f;
                float bb = b ? get_float(b) : 0.0f;
                self->out.b = (aa >= bb);
                break;
            }



            case PLC_NODE_DIGITAL_OUT: {
                bool desired = get_bool(a);
                if (self->force_en && (self->flags & PLC_NODE_FLAG_FORCE)) {
                    desired = self->force_bool;
                }
                hw_write_discrete_output(self->paramInt, desired);
                self->out.b = desired;

#if PLC_LOG_ENABLED && PLC_LOG_GRAPH
                if (!s_do_init[i] || s_do_last[i] != (uint8_t) desired) {
                    s_do_init[i] = 1;
                    s_do_last[i] = (uint8_t) desired;
                    LOGT(PLC_LOG_TAG, "DO[%u] ch=%d -> %u (force=%u)",
                         (unsigned) i, (int) self->paramInt, (unsigned) desired, (unsigned) self->force_en);
                }
#endif
                break;
            }


            case PLC_NODE_PWM_OUT: {
                int32_t duty = get_int(a);

                if (duty < 0) duty = 0;
                if (duty > 100) duty = 100;

                // FORCE
                if (self->force_en && (self->flags & PLC_NODE_FLAG_FORCE)) {
                    duty = self->force_bool ? 100 : 0;
                    self->pwm_zero_hold = false; // форс игнорирует deadband
                } else {
                    // Deadband + hysteresis near zero
                    if (!self->pwm_zero_hold) {
                        // если мы "в норме" и пришли в зону выключения -> зафиксировать 0
                        if (duty <= PLC_PWM_DEADBAND_OFF_PCT) {
                            self->pwm_zero_hold = true;
                        }
                    } else {
                        // если мы в "нулевой фиксации", включаем только когда перешли порог включения
                        if (duty >= PLC_PWM_DEADBAND_ON_PCT) {
                            self->pwm_zero_hold = false;
                        }
                    }

                    if (self->pwm_zero_hold) duty = 0;
                }

                hw_write_pwm_output(self->paramInt, duty);
                self->out.i = duty;
                self->out.f = (float) duty;

#if PLC_LOG_ENABLED && PLC_LOG_GRAPH
                if (!s_pwm_init[i] || iabs32(duty - s_pwm_last[i]) >= PLC_PWM_DELTA) {
        LOGT(PLC_LOG_TAG, "PWM[%u] ch=%d -> %ld%% (force=%u hold0=%u)",
             (unsigned)i, (int)self->paramInt, (long)duty,
             (unsigned)self->force_en, (unsigned)self->pwm_zero_hold);
        s_pwm_init[i] = 1;
        s_pwm_last[i] = duty;
    }
#endif
                break;
            }


            case PLC_NODE_AO: {
                // Вход как float: ожидаем диапазон 0..100 (%)
                float v = a ? get_float(a) : 0.0f;

                if (v < 0.0f)   v = 0.0f;
                if (v > 100.0f) v = 100.0f;

                // FORCE
                if (self->force_en && (self->flags & PLC_NODE_FLAG_FORCE)) {
                    v = self->force_bool ? 100.0f : 0.0f;
                    self->pwm_zero_hold = false;
                } else {
                    // Deadband + hysteresis for AO (percent)
                    if (!self->pwm_zero_hold) {
                        if (v <= (float)PLC_PWM_DEADBAND_OFF_PCT) {
                            self->pwm_zero_hold = true;
                        }
                    } else {
                        if (v >= (float)PLC_PWM_DEADBAND_ON_PCT) {
                            self->pwm_zero_hold = false;
                        }
                    }

                    if (self->pwm_zero_hold) v = 0.0f;
                }

                int32_t duty = (int32_t)(v + 0.5f); // округление 0..100

                hw_write_pwm_output(self->paramInt, duty);

                self->out.f = v;

#if PLC_LOG_ENABLED && PLC_LOG_GRAPH
                if (!s_pwm_init[i] || iabs32(duty - s_pwm_last[i]) >= PLC_PWM_DELTA) {
        LOGT(PLC_LOG_TAG, "AO[%u] ch=%d -> %.1f%% (duty=%ld, force=%u hold0=%u)",
             (unsigned)i, (int)self->paramInt, (double)v,
             (long)duty, (unsigned)self->force_en, (unsigned)self->pwm_zero_hold);
        s_pwm_init[i] = 1;
        s_pwm_last[i] = duty;
    }
#endif
                break;
            }



            case PLC_NODE_MUX2: {
                const PlcNode *s = (self->paramInt >= 0) ? &g->nodes[self->paramInt] : NULL;
                bool sel = s ? get_bool(s) : false;

                switch (self->valueType) {
                    case PLC_VAL_BOOL: {
                        bool va = a ? get_bool(a) : false;
                        bool vb = b ? get_bool(b) : false;
                        bool y = sel ? va : vb;
                        self->out.b = y;
                        break;
                    }

                    case PLC_VAL_INT: {
                        int32_t va = a ? get_int(a) : 0;
                        int32_t vb = b ? get_int(b) : 0;
                        int32_t y = sel ? va : vb;
                        self->out.i = y;
                        break;
                    }

                    case PLC_VAL_FLOAT:
                    default: {
                        float va = a ? get_float(a) : 0.0f;
                        float vb = b ? get_float(b) : 0.0f;
                        float y = sel ? va : vb;
                        self->out.f = y;
                        break;
                    }
                }

#if PLC_LOG_ENABLED && PLC_LOG_GRAPH
                static uint8_t s_mux_init[PLC_MAX_NODES];
                static uint8_t s_mux_last_sel[PLC_MAX_NODES];
                if (!s_mux_init[i] || s_mux_last_sel[i] != (uint8_t) sel) {
                    s_mux_init[i] = 1;
                    s_mux_last_sel[i] = (uint8_t) sel;
                    if (self->valueType == PLC_VAL_FLOAT)
                    LOGT(PLC_LOG_TAG,  "MUX2[%u] sel=%u -> y=%.3f", i, sel, (double)self->out.f);
                    else if (self->valueType == PLC_VAL_INT)
                    LOGT(PLC_LOG_TAG,  "MUX2[%u] sel=%u -> y=%ld", i, sel, (long)self->out.i);
                    else
                    LOGT(PLC_LOG_TAG,  "MUX2[%u] sel=%u -> y=%u", i, sel, (unsigned)self->out.b);
                }
#endif
                break;
            }

            case PLC_NODE_TP: {
                bool a_now = a ? get_bool(a) : false;
                bool rising = (a_now && !self->trig_prev);
                self->trig_prev = a_now;

                if (rising) {
                    self->toff_holding = true;
                    self->toff_left_ms = self->paramMs;
#if PLC_LOG_ENABLED && PLC_LOG_GRAPH
                    LOGT(PLC_LOG_TAG, "TP[%u] start T=%u", (unsigned) i, (unsigned) self->paramMs);
#endif
                }

                bool pulse = self->toff_holding;

                if (self->toff_holding) {
                    uint32_t step = dt_ms;
                    if (step >= self->toff_left_ms) {
                        self->toff_left_ms = 0;
                        self->toff_holding = false;
#if PLC_LOG_ENABLED && PLC_LOG_GRAPH
                        LOGT(PLC_LOG_TAG, "TP[%u] end", (unsigned) i);
#endif
                    } else {
                        self->toff_left_ms -= step;
                    }
                }

                self->out.b = pulse;
                break;
            }


            case PLC_NODE_HYST: {
                float x = a ? get_float(a) : 0.0f;
                float center = self->paramFloat;
                float h = (float) self->paramInt * 0.001f;
                if (h < 0.0f) h = -h;
                float hi = center + 0.5f * h;
                float lo = center - 0.5f * h;

                bool y = self->sr_q;
                if (x >= hi) y = true; else if (x <= lo) y = false;

                bool prev = self->sr_q;
                self->sr_q = y;
                self->out.b = y;

#if PLC_LOG_ENABLED && PLC_LOG_GRAPH
                if (y != prev) {
                    const char* why = (x >= hi) ? "x>=hi => ON" : (x <= lo) ? "x<=lo => OFF" : "hold";
                    LOGT(PLC_LOG_TAG,
                         "HYST[%u] %s | x=%.3f prev=%u -> %u | lo=%.3f hi=%.3f (center=%.3f h=%.3f)",
                         (unsigned)i, why,
                         (double)x, (unsigned)prev, (unsigned)y,
                         (double)lo, (double)hi,
                         (double)center, (double)h);
                }
#endif
                break;
            }

            case PLC_NODE_SCALE: {
                float x = a ? get_float(a) : 0.0f;
                float A = self->paramFloat;
                float B = (float) self->paramInt * 0.001f;
                float y = A * x + B;
                self->out.f = y;
                break;
            }
            case PLC_NODE_ADD: {
                float va = a ? get_float(a) : 0.0f;
                float vb = b ? get_float(b) : 0.0f;
                float y = va + vb;
                self->out.f = y;
                break;
            }
            case PLC_NODE_LIMIT: {
                float x = a ? get_float(a) : 0.0f;
                float mn = (float) self->paramInt * 0.001f;
                float mx = (float) self->paramMs * 0.001f;
                if (mn > mx) {
                    float t = mn;
                    mn = mx;
                    mx = t;
                }
                float y = (x < mn) ? mn : (x > mx ? mx : x);
                self->out.f = y;
                break;
            }

            case PLC_NODE_PID: {
                // Входы
                float sp = a ? get_float(a) : 0.0f;   // Setpoint
                float pv = b ? get_float(b) : 0.0f;   // Process Value

                // Параметры
                const float Kp = self->paramFloat;                      // как есть
                const float Ki = (float) self->paramInt * 0.001f;        // тысячные → float
                const float Kd = (float) self->paramMs * 0.001f;        // тысячные → float

                // Δt, сек
                const float Ts = (dt_ms > 0) ? ((float) dt_ms * 0.001f) : 0.001f;

                // Ошибка
                const float e = sp - pv;

                // Инициализация
                if (!self->pid_inited) {
                    self->pid_i = 0.0f;
                    self->pid_prev_meas = pv;
                    self->pid_inited = true;
                }

                // Интегральная часть: прямоугольная интеграция
                float i_next = self->pid_i + Ki * e * Ts;

                // Производная по измерению (anti-kick): d = -Kd * d(PV)/dt
                float d_meas = (pv - self->pid_prev_meas) / Ts;
                float d_term = -Kd * d_meas;
                self->pid_prev_meas = pv;

                // Пропорциональная часть
                float p_term = Kp * e;

                // Сумма до насыщения
                float u = p_term + i_next + d_term;

                // Внутренний кламп (опционально) + простой anti-windup
                if (self->flags & PLC_NODE_FLAG_PID_CLAMP_0_100) {
                    if (u > 100.0f) {
                        u = 100.0f;
                        // Если ошибка «толкает» наружу — не увеличиваем интеграл
                        if (e > 0.0f) i_next = self->pid_i;
                    } else if (u < 0.0f) {
                        u = 0.0f;
                        if (e < 0.0f) i_next = self->pid_i;
                    }
                } else if (self->flags & PLC_NODE_FLAG_PID_CLAMP_0_1) {
                    if (u > 1.0f) {
                        u = 1.0f;
                        if (e > 0.0f) i_next = self->pid_i;
                    } else if (u < 0.0f) {
                        u = 0.0f;
                        if (e < 0.0f) i_next = self->pid_i;
                    }
                }
                self->pid_i = i_next;

                // --- Выход по valueType ---
                switch (self->valueType) {
                    case PLC_VAL_BOOL:
                        // "выше нуля" = включено
                        self->out.b = (u != 0.0f);
                        break;

                    case PLC_VAL_INT:
                        // Можно округлять:
                        // self->out.i = (int32_t)lroundf(u);
                        self->out.i = (int32_t)u;
                        break;

                    case PLC_VAL_FLOAT:
                    default:
                        self->out.f = u;
                        break;
                }
                break;
            }

            case PLC_NODE_ANALOG_AVG: {
                float x = a ? get_float(a) : 0.0f;
                float alpha = (float) self->paramInt * 0.001f; // 0..1
                if (!self->filt_inited) {
                    self->filt_prev = x;
                    self->filt_inited = true;
                }
                float y = self->filt_prev + alpha * (x - self->filt_prev);
                self->filt_prev = y;
                self->out.f = y;
                break;
            }
            case PLC_NODE_RAMP: {
                float x = a ? get_float(a) : 0.0f;
                float rate = (float) self->paramInt * 0.001f; // ед/с
                float Ts = (dt_ms > 0) ? (0.001f * dt_ms) : 0.001f;
                float max_delta = rate * Ts; // допустимое изменение за шаг

                if (!self->ramp_inited) {
                    self->ramp_prev = x;
                    self->ramp_inited = true;
                }
                float delta = x - self->ramp_prev;
                if (delta > max_delta) delta = max_delta;
                if (delta < -max_delta) delta = -max_delta;

                float y = self->ramp_prev + delta;
                self->ramp_prev = y;
                self->out.f = y;
                break;
            }
            case PLC_NODE_LOG: {
                // Просто пробрасываем вход на выход (чтобы LOG можно было "подцеплять" в цепи)
                float x = a ? get_float(a) : 0.0f;
                self->out.f = x;

                // Накопление времени
                self->log_accum_ms += dt_ms;
                if (self->log_accum_ms >= self->paramMs) {
                    self->log_accum_ms = 0;

                    // Кодируем значение грубо в int16 для журнала (например, x*100)
                    int16_t v = 0;
                    if (x > 327.67f)  v = 32767;
                    else if (x < -327.68f) v = -32768;
                    else v = (int16_t)(x * 100.0f);

                    // a = nodeId, b = значение*100
                    plc_log_put(PLC_EVT_LOG_VALUE, (int16_t)self->id, v);
                }
                break;
            }

            case PLC_NODE_CTU: {
                bool clk = get_bool(a);
                bool rst = (b ? get_bool(b) : false);

                if (rst) {
                    self->acc = 0;
                }

                bool rising = (clk && !self->prev_clk);
                self->prev_clk = clk;

                if (rising) {
                    if (self->acc < INT32_MAX) {
                        self->acc++;
                    }
                }

                if (self->acc < 0)        self->acc = 0;
                if (self->acc > INT32_MAX) self->acc = INT32_MAX;

                // CV
                self->out.i = self->acc;

//                // Q: достигнут порог PV (если задан)
//                if (self->paramInt > 0) {
//                    self->out.b = (self->acc >= self->paramInt);
//                } else {
//                    // Если PV == 0, можно считать Q = (acc > 0) или всегда false — на твой вкус
//                    self->out.b = (self->acc > 0);
//                }

                break;
            }
            case PLC_NODE_CTD: {
                bool clk = get_bool(a);
                bool rst = (b ? get_bool(b) : false);

                // RST загружает preset
                if (rst) {
                    self->acc = self->paramInt;
                    if (self->acc < 0) self->acc = 0;
                }

                bool rising = (clk && !self->prev_clk);
                self->prev_clk = clk;

                if (rising && self->acc > 0) {
                    self->acc--;
                }

                if (self->acc < 0)        self->acc = 0;
                if (self->acc > INT32_MAX) self->acc = INT32_MAX;

                self->out.i = self->acc;

                // Q: счётчик «дошёл/ниже порога».
                // Например, считаем, что Q=1 когда acc == 0.
//                if (self->paramInt > 0) {
//                    // вариант: Q = (acc <= 0) или (acc <= paramInt), как тебе удобнее
//                    self->out.b = (self->acc <= 0);
//                } else {
//                    self->out.b = (self->acc <= 0);
//                }

                break;
            }
            case PLC_NODE_CTUD: {
                bool cu = get_bool(a); // inA = CU
                bool cd = get_bool(b); // inB = CD

                bool clk_any = (cu || cd);
                bool rising  = (clk_any && !self->prev_clk);
                self->prev_clk = clk_any;

                if (rising) {
                    if (cu && !cd) {
                        if (self->acc < INT32_MAX) self->acc++;
                    } else if (cd && !cu) {
                        if (self->acc > 0) self->acc--;
                    }
                    // cu && cd одновременно — игнорируем
                }

                if (self->acc < 0)        self->acc = 0;
                if (self->acc > INT32_MAX) self->acc = INT32_MAX;

                self->out.i = self->acc;

//                if (self->paramInt > 0) {
//                    self->out.b = (self->acc >= self->paramInt);
//                } else {
//                    self->out.b = (self->acc > 0);
//                }

                break;
            }

                // --------- Новые узлы ---------

            case PLC_NODE_WINDOW_CHECK: {
                // x — как float (int тоже нормально сконвертируется)
                float x = a ? get_float(a) : 0.0f;
                float center = self->paramFloat;
                float width  = (float)self->paramInt * 0.001f; // в тех же единицах, что x

                float lo = center - width * 0.5f;
                float hi = center + width * 0.5f;

                self->out.b = (x >= lo && x <= hi);
                break;
            }

            case PLC_NODE_SAFE_OUTPUT: {
                bool allow = b ? get_bool(b) : true;  // если нет входа B — считаем, что всегда разрешено

                switch (self->valueType) {
                    case PLC_VAL_BOOL: {
                        bool normal = a ? get_bool(a) : false;
                        bool safe   = (self->paramInt != 0); // safe=1 или 0
                        self->out.b = allow ? normal : safe;
                        break;
                    }

                    case PLC_VAL_INT: {
                        int32_t normal = a ? get_int(a) : 0;
                        int32_t safe   = self->paramInt;
                        self->out.i = allow ? normal : safe;
                        break;
                    }

                    case PLC_VAL_FLOAT:
                    default: {
                        float normal = a ? get_float(a) : 0.0f;
                        float safe   = self->paramFloat; // безопасное значение
                        float y      = allow ? normal : safe;
                        self->out.f = y;
                        break;
                    }
                }
                break;
            }

            case PLC_NODE_ALARM_GEN: {
                bool cond = a ? get_bool(a) : false;
                self->out.b = cond;
                break;
            }

            case PLC_NODE_ALARM_LATCH: {
                bool cond = a ? get_bool(a) : false;
                bool ack  = b ? get_bool(b) : false;

                if (ack) {
                    self->sr_q = false;
                } else if (cond) {
                    self->sr_q = true;
                }
                self->out.b = self->sr_q;
                break;
            }

            case PLC_NODE_HEARTBEAT: {
                // Мигаем с периодом paramMs, используем log_accum_ms как накопитель
                self->log_accum_ms += dt_ms;
                if (self->log_accum_ms >= self->paramMs) {
                    self->log_accum_ms -= self->paramMs;
                    self->out.b = !self->out.b;
                }
                break;
            }
            case PLC_NODE_MEM_BOOL: {
                uint16_t idx = (uint16_t) self->paramInt;
                if (idx >= PLC_MEM_BOOL_COUNT) {
                    // не должен сюда попасть, валидация уже отфильтровала, но на всякий случай
                    self->out.b = false;
                    break;
                }

                bool val = plc_mem_get_bool(idx);

                // Если есть вход A — это источник нового значения
                if (self->inA >= 0 && a) {
                    // Если есть вход B — используем как WE (write enable)
                    bool we = true;
                    if (self->inB >= 0 && b) {
                        we = get_bool(b);
                    }
                    if (we) {
                        val = get_bool(a);
                        plc_mem_set_bool(idx, val);
                    }
                }

                self->out.b = val;
                break;
            }

            case PLC_NODE_MEM_INT: {
                uint16_t idx = (uint16_t) self->paramInt;
                if (idx >= PLC_MEM_INT_COUNT) {
                    self->out.i = 0;
                    break;
                }

                int32_t val = plc_mem_get_int(idx);

                if (self->inA >= 0 && a) {
                    bool we = true;
                    if (self->inB >= 0 && b) {
                        we = get_bool(b);
                    }
                    if (we) {
                        val = get_int(a);
                        plc_mem_set_int(idx, val);
                    }
                }

                self->out.i = val;
                break;
            }

            case PLC_NODE_MEM_REAL: {
                uint16_t idx = (uint16_t) self->paramInt;
                if (idx >= PLC_MEM_REAL_COUNT) {
                    self->out.f = 0.0f;
                    break;
                }

                float val = plc_mem_get_real(idx);

                if (self->inA >= 0 && a) {
                    bool we = true;
                    if (self->inB >= 0 && b) {
                        we = get_bool(b);
                    }
                    if (we) {
                        val = get_float(a);
                        plc_mem_set_real(idx, val);
                    }
                }

                self->out.f = val;
                break;
            }
            case PLC_NODE_FILTER_AVG: {
                // Вход как float (int тоже конвертируется через get_float)
                float x = a ? get_float(a) : 0.0f;

                // alpha в [0..1.0]
                float alpha = (float)self->paramInt * 0.001f;
                if (alpha <= 0.0f) {
                    // alpha=0 -> "заморозить" фильтр, просто держим текущее значение out.f
                    // можно также сделать out.f = x, если нужно мгновенное следование
                    // сейчас оставляем как есть
                    // self->out.f = self->out.f;
                } else if (alpha >= 1.0f) {
                    // alpha=1 -> без фильтрации
                    self->out.f = x;
                } else {
                    float y_prev = self->out.f;
                    float y = y_prev + alpha * (x - y_prev);
                    self->out.f = y;
                }
                break;
            }
            case PLC_NODE_MATH_OP: {
                int op = self->paramInt; // 0=ADD,1=SUB,2=MUL,3=DIV

                switch (self->valueType) {
                    case PLC_VAL_INT: {
                        int32_t va = a ? get_int(a) : 0;
                        int32_t vb = b ? get_int(b) : 0;
                        int32_t r = 0;

                        switch (op) {
                            case 0: // ADD
                                r = va + vb;
                                break;
                            case 1: // SUB
                                r = va - vb;
                                break;
                            case 2: // MUL
                                r = va * vb;
                                break;
                            case 3: // DIV
                                if (vb == 0) {
                                    // деление на 0: оставим предыдущее значение
                                    r = self->out.i;
                                } else {
                                    r = va / vb;
                                }
                                break;
                            default:
                                r = 0;
                                break;
                        }

                        self->out.i = r;
                        break;
                    }
                    case PLC_VAL_FLOAT:
                    default: {
                        float va = a ? get_float(a) : 0.0f;
                        float vb = b ? get_float(b) : 0.0f;
                        float r  = 0.0f;

                        switch (op) {
                            case 0: // ADD
                                r = va + vb;
                                break;
                            case 1: // SUB
                                r = va - vb;
                                break;
                            case 2: // MUL
                                r = va * vb;
                                break;
                            case 3: // DIV
                                if (vb == 0.0f) {
                                    // деление на 0: оставим предыдущее значение
                                    r = self->out.f;
                                } else {
                                    r = va / vb;
                                }
                                break;
                            default:
                                r = 0.0f;
                                break;
                        }

                        self->out.f = r;
                        break;
                    }
                }

                break;
            }
            case PLC_NODE_CONST_FLOAT: {
                self->out.f = self->paramFloat;
                break;
            }

            default:
                // неизвестный тип
                break;
        }
    }
}




// --- Реализация API доступа к памяти ПЛК ---

void plc_mem_set_bool(uint16_t index, bool value) {
    if (index >= PLC_MEM_BOOL_COUNT) return;
    s_memBool[index] = value;
}

bool plc_mem_get_bool(uint16_t index) {
    if (index >= PLC_MEM_BOOL_COUNT) return false;

    bool v;
    v = s_memBool[index];
    return v;
}

void plc_mem_set_int(uint16_t index, int32_t value) {
    if (index >= PLC_MEM_INT_COUNT) return;
    s_memInt[index] = value;
}

int32_t plc_mem_get_int(uint16_t index) {
    if (index >= PLC_MEM_INT_COUNT) return 0;

    int32_t v;
    v = s_memInt[index];
    return v;
}

void plc_mem_set_real(uint16_t index, float value) {
    if (index >= PLC_MEM_REAL_COUNT) return;
    s_memReal[index] = value;
}

float plc_mem_get_real(uint16_t index) {
    if (index >= PLC_MEM_REAL_COUNT) return 0.0f;

    float v;
    v = s_memReal[index];
    return v;
}

void plc_mem_reset_all(void)
{
    memset(s_memBool, 0, sizeof(s_memBool));
    memset(s_memInt,  0, sizeof(s_memInt));
    memset(s_memReal, 0, sizeof(s_memReal));
}

void plc_mem_init(void)
{
    plc_mem_reset_all(); // пока RAM-only
}


