#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define ADC_CH_COUNT 4
#define DMA_ALIGN 32
// Лучше в D2 SRAM, если используешь D-Cache:
// GCC атрибут зависит от линкера, но минимум выровнять:
extern volatile uint32_t g_adcRaw[4];

/* ================== Константы и типы ================== */
#define PLC_MAX_NODES 100          // подбери под свою память
#define PLC_CYCLE_MS_MIN 5        // защита от слишком частого цикла
#define PLC_CYCLE_MS_MAX 100      // защита от слишком редкого цикла

#define PLC_NODE_FLAG_PID_CLAMP_0_100   (1u<<8)   // выход 0..100
#define PLC_NODE_FLAG_PID_CLAMP_0_1     (1u<<9)   // выход 0..1

// --- Размеры таблиц памяти ПЛК (можешь потом увеличить) ---
#define PLC_MEM_BOOL_COUNT  32
#define PLC_MEM_INT_COUNT   32
#define PLC_MEM_REAL_COUNT  32

// ===== PWM/AO deadband =====
// Сколько процентов считать "нулём"
#ifndef PLC_PWM_DEADBAND_OFF_PCT
#define PLC_PWM_DEADBAND_OFF_PCT   2   // если <=2% -> считаем 0
#endif

// Гистерезис включения: чтобы после "выкл" включалось не сразу
#ifndef PLC_PWM_DEADBAND_ON_PCT
#define PLC_PWM_DEADBAND_ON_PCT    3   // включать только если >=3%
#endif

#define PLC_RESET_MEM_ON_ACTIVATE 0

typedef enum {
    PLC_VAL_BOOL  = 0,  // Логическое значение: 0 или 1 (дискретные сигналы, кнопки, реле, флаги)
    PLC_VAL_INT   = 1,  // Целое число (обычно 32-битное): используется для счётчиков, порогов, параметров
    PLC_VAL_FLOAT = 2   // Вещественное число (float): аналоговые данные, температура, напряжение, коэффициенты
} PlcValueType;

typedef enum {
    PLC_NODE_CONST_BOOL = 0,  // Постоянное логическое значение (0 или 1), используется как источник TRUE/FALSE
    PLC_NODE_CONST_INT  = 1,  // Постоянное целое число (уставка, порог, коэффициент)
    PLC_NODE_DIGITAL_IN = 2,  // Дискретный вход (чтение GPIO, кнопки, датчики)
    PLC_NODE_DIGITAL_OUT= 3,  // Дискретный выход (управление GPIO, лампы, реле)
    PLC_NODE_AND2       = 4,  // Логическое "И" (выход = 1, если оба входа = 1)
    PLC_NODE_OR2        = 5,  // Логическое "ИЛИ" (выход = 1, если хотя бы один вход = 1)
    PLC_NODE_NOT        = 6,  // Логическое "НЕ" (инвертирует сигнал: 1→0, 0→1)
    PLC_NODE_SR         = 7,  // Защёлка (Set/Reset latch): сохраняет состояние между циклами
    PLC_NODE_TON        = 8,  // Таймер на включение (Delay ON): срабатывает после удержания входа заданное время
    PLC_NODE_TOFF       = 9,  // Таймер на выключение (Delay OFF): задерживает выключение после отпускания входа
    PLC_NODE_R_TRIG     = 10, // Импульс по фронту (Rising trigger): 1 цикл при переходе 0→1
    PLC_NODE_F_TRIG     = 11, // Импульс по спаду (Falling trigger): 1 цикл при переходе 1→0
    PLC_NODE_AI_IN      = 12, // Аналоговый вход (чтение АЦП, датчиков, потенциометров)
    PLC_NODE_COMPARE_GT = 13, // Сравнение "БОЛЬШЕ" (A > B)
    PLC_NODE_COMPARE_LT = 14, // Сравнение "МЕНЬШЕ" (A < B)
    PLC_NODE_PWM_OUT    = 15, // ШИМ-выход (PWM): формирование аналогового сигнала по коэффициенту заполнения
    PLC_NODE_MUX2       = 16, // селектор: out = sel ? inA : inB; sel берём из node[paramInt]
    PLC_NODE_TP         = 17, // одноимпульсный таймер: фронт inA -> импульс paramMs
    PLC_NODE_HYST       = 18, // гистерезис: inA(float)->bool; центр=paramFloat; ширина=paramInt/1000
    PLC_NODE_SCALE      = 19, // y=a*x+b; a=paramFloat; b=paramInt/1000
    PLC_NODE_ADD        = 20, // out = inA + inB (float)
    PLC_NODE_LIMIT      = 21, // clamp: min=paramInt/1000; max=paramMs/1000
    PLC_NODE_PID        = 22,
    PLC_NODE_ANALOG_AVG = 23, // сглаживание: y += alpha*(x - y); alpha = paramInt/1000
    PLC_NODE_RAMP       = 24, // ограничение скорости: |y - prev| <= rate*dt; rate = paramInt/1000 ед/с
    PLC_NODE_LOG        = 25, // лог: периодически пишет значение входа в plc_log
    PLC_NODE_AO         = 26, // Аналоговый выход (0..100%) -> ШИМ/DAC по номеру канала
    PLC_NODE_CTU        = 27, // Счётчик вверх: inA=CLK, inB=RST, paramInt=PV (preset)
    PLC_NODE_CTD        = 28, // Счётчик вниз: фронт inA (CLK) -> acc--, inB (RST) загружает preset из paramInt
    PLC_NODE_CTUD       = 29, // Счётчик вверх/вниз: inA = CU, inB = CD (пульсы); out.i = acc
    PLC_NODE_WINDOW_CHECK = 30, // Проверка диапазона: MIN..MAX вокруг paramFloat (центр), ширина paramInt/1000
    PLC_NODE_SAFE_OUTPUT  = 31, // Безопасный выход: нормальное управление + безопасное состояние при аварии
    PLC_NODE_ALARM_GEN    = 32, // Генерация аварийного флага (обёртка над условием)
    PLC_NODE_ALARM_LATCH  = 33, // Защёлкивающаяся авария: держит 1 до квитирования
    PLC_NODE_HEARTBEAT    = 34, // "Сердцебиение": периодическое мигание для проверки живости ПЛК
    // Узлы памяти (внутренние регистры)
    PLC_NODE_MEM_BOOL     = 35, // Логическая переменная: чтение/запись bool по индексу
    PLC_NODE_MEM_INT      = 36, // Целочисленная переменная: чтение/запись int32
    PLC_NODE_MEM_REAL     = 37, // Вещественная переменная: чтение/запись float

    PLC_NODE_FILTER_AVG   = 38, // Экспоненциальный фильтр: сглаживание аналогового сигнала
    PLC_NODE_MATH_OP      = 39, // Базовые операции: +, -, *, / над двумя входами
    PLC_NODE_CONST_FLOAT  = 40,  // константа float (уставки, коэффициенты)
    PLC_NODE_COMPARE_GE   = 41, // A >= B
    PLC_NODE_MAX = PLC_NODE_COMPARE_GE
} PlcNodeType;

// --- Доступ к памяти ПЛК (глобальные регистры) ---
void     plc_mem_set_bool(uint16_t index, bool value);
bool     plc_mem_get_bool(uint16_t index);

void     plc_mem_set_int(uint16_t index, int32_t value);
int32_t  plc_mem_get_int(uint16_t index);

void     plc_mem_set_real(uint16_t index, float value);
float    plc_mem_get_real(uint16_t index);

void plc_mem_reset_all(void);
void plc_mem_init(void);

// ──────────────────────────────────────────────────────────────
// Флаг для поля flags (битовые настройки узла)
// ──────────────────────────────────────────────────────────────
#define PLC_NODE_FLAG_FORCE   (1u << 0)  // Разрешает принудительное (ручное) управление выходом узла

// ──────────────────────────────────────────────────────────────
// Основная структура описания одного узла (Node) в графе ПЛК
// ──────────────────────────────────────────────────────────────
typedef struct {
    // ────────────────────────────────
    // Статическая часть — приходит по CBOR при загрузке графа
    // ────────────────────────────────

    uint16_t    id;         // Уникальный индекс узла в графе (0..65535)
    uint8_t     type;       // Тип узла (см. PlcNodeType — AND, TON, PWM и т.д.)
    uint8_t     valueType;  // Тип данных (см. PlcValueType — BOOL, INT, FLOAT)

    int16_t     inA;        // Индекс узла-источника A; -1 = вход не подключен
    int16_t     inB;        // Индекс узла-источника B; -1 = вход не подключен

    int32_t     paramInt;   // Целочисленный параметр (порог, коэффициент, номер пина и т.п.)
    float       paramFloat; // Вещественный параметр (уставка, коэффициент масштабирования)
    uint32_t    paramMs;    // Параметр времени (мс) — для таймеров TON/TOFF
    uint32_t    flags;      // Битовые флаги поведения узла (например, PLC_NODE_FLAG_FORCE и др.)

    // ────────────────────────────────
    // Динамическая часть — формируется и изменяется во время работы
    // ────────────────────────────────

    struct {
        bool    b;  // Логическое значение (для дискретных узлов)
        int32_t i;  // Целое значение (для счётчиков, арифметики)
        float   f;  // Вещественное значение (для аналоговых сигналов)
    } out;          // Текущее вычисленное значение узла (результат работы)

    // ────────────────────────────────
    // Внутреннее состояние для "памятных" блоков
    // ────────────────────────────────

    bool     sr_q;            // Хранит состояние для блока SR (Set/Reset latch)
    bool     trig_prev;       // Предыдущее состояние входа для R_TRIG / F_TRIG (детектор фронта)

    bool     ton_active;      // TON: активен ли отсчёт таймера
    uint32_t ton_accum_ms;    // TON: накопленное время при удержании входа

    bool     toff_holding;    // TOFF: в фазе удержания после отпускания входа
    uint32_t toff_left_ms;    // TOFF: сколько миллисекунд осталось до выключения

    // ────────────────────────────────
    // Принудительный (ручной) режим управления выходом
    // ────────────────────────────────

    bool     force_en;        // Включён ли ручной режим (игнорировать входы)
    bool     force_bool;      // Принудительное значение (0/1), удерживаемое в ручном режиме
    uint32_t force_left_ms;   // Сколько миллисекунд ещё держать принудительное состояние

    /* --- PID runtime --- */
    float    pid_i;          // интегральная составляющая
    float    pid_prev_meas;  // предыдущее PV для d(PV)/dt
    bool     pid_inited;     // флаг инициализации

    // для обоих узлов достаточно одного буфера предыдущего значения
    float  filt_prev;     // для ANALOG_AVG: предыдущее y
    bool   filt_inited;   // инициализация фильтра
    float  ramp_prev;     // для RAMP: предыдущий выход
    bool   ramp_inited;

    uint32_t log_accum_ms;   // накопленное время с последней записи

    //PLC_NODE_CTU states:
    int32_t acc;
    bool prev_clk;

    // Deadband state for PWM/AO
    bool pwm_zero_hold;  // держим выход в 0 из-за deadband
} PlcNode;

typedef struct {
    uint32_t cycleMs;              // период цикла ПЛК (желаемый)
    uint16_t nodeCount;
    PlcNode  nodes[PLC_MAX_NODES];
} PlcGraph;

/* ================== Глобальные графы ================== */

extern PlcGraph g_activeGraph;
extern PlcGraph g_stagingGraph;

extern bool g_activeGraphValid;
extern bool g_stagingGraphValid;

extern volatile bool g_needSwapGraph;

/* ================== Команды из TaskComms -> TaskPlcScan ================== */

typedef enum {
    CMD_FORCE_OUTPUT = 1,
    CMD_SET_NODE_PARAM = 2,
    // можно расширять
} PlcCmdType;

// команда на ПЛК-движок
typedef struct {
    PlcCmdType type;
    uint16_t   nodeIndex; // к какому узлу относится (например DIGITAL_OUT)
    uint32_t   value;     // значение (например 1/0 или новое значение порога)
    uint32_t   holdMs;    // для FORCE_OUTPUT: сколько мс держать
} PlcCmd;

// Очередь команд
//extern QueueHandle_t plcCmdQueue;

/* ================== Публичные функции ПЛК ================== */

// вызывается TaskComms при получении upload_graph
bool plc_upload_graph(const PlcGraph* src);

// вызывается TaskComms при получении activate_graph
bool plc_request_activate_graph(void);

// принудительное управление выходом (DO / PWM)
// nodeIndex — индекс узла в активном графе
// value     — 0/1 (false/true)
// holdMs    — сколько миллисекунд держать принудительное состояние
bool plc_force_output(uint16_t nodeIndex, bool value, uint32_t holdMs);

// снять форс, вернуть узел под управление графа
bool plc_release_output(uint16_t nodeIndex);

// основной тик ПЛК, вызывается TaskPlcScan в цикле
void plc_tick(uint32_t nowMs);

void plc_metrics_on_cycle(uint32_t execMs, uint32_t periodMs);

// сервисные куски (ниже будут реализованы в plc.c)
void plc_refresh_inputs_hw(uint32_t nowMs, PlcGraph *g);
void plc_apply_pending_cmds(PlcGraph* g, uint32_t dt_ms);
void plc_graph_step(PlcGraph* g, uint32_t dt_ms);

// внутренние вспомогательные
void plc_graph_reset_runtime(PlcGraph* g);

bool plc_validate_graph(PlcGraph* g);

/* аппаратно-зависимые функции, которые ты реализуешь под свою плату */
bool     hw_read_discrete_input(int logicalCh);
void     hw_write_discrete_output(int logicalCh, bool val);
int32_t  hw_read_adc_input(int logicalCh);
void     hw_write_pwm_output(int logicalCh, int dutyPercent);

uint32_t now_ms(void);     // системное время мс
void     iwdg_feed(void);  // кормим watchdog

/* Задачи RTOS (реализуются в tasks.c) */
/* TaskPlcScan:
   Вызывает plc_tick().
   Делает DelayUntil по cycleMs activeGraph.
*/
void TaskPlcScan(void* arg);

void UartTx_Task(void *arg);  // создать один раз

/* TaskComms:
   Читает кадры по UART и передаёт в handle_frame_from_host().
*/
void TaskComms(void* arg);

void TaskInitial(void* arg);

void TaskInitial2(void* arg);

/* Функция init_rtos() вызывается из main.c
   после HAL_Init, настройки GPIO/UART/ADC/TIM/IWDG:
   - создаёт plcCmdQueue
   - создаёт таски
   - запускает планировщик
*/
void init_rtos(void);

void plc_hw_reset_outputs(void);

// ---- PLC runtime metrics (microseconds) ----
extern uint32_t g_scan_cnt;
extern uint64_t g_scan_us_accum;
extern uint32_t g_scan_us_max;
extern uint32_t g_scan_long_steps;

// call from TaskPlcScan once per cycle
void plc_metrics_on_cycle_us(uint32_t execUs, uint32_t periodMs);

extern uint32_t g_work_cnt;
extern uint64_t g_work_us_accum;
extern uint32_t g_work_us_max;

extern uint32_t g_cycle_cnt;
extern uint64_t g_cycle_ms_accum;
extern uint32_t g_cycle_ms_max;

void plc_metrics_on_work_us(uint32_t workUs);
void plc_metrics_on_cycle_ms(uint32_t realCycleMs);


#ifdef __cplusplus
}
#endif