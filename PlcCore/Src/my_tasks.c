// my_tasks.c (CMSIS-RTOS2 + FreeRTOS primitives where needed)

#include "cmsis_os2.h"

#include "main.h"
#include "usb_device.h"
#include "lwip.h"

#include "plc.h"
#include "plc_comm_tcp.h"
#include "cbor_msg.h"

#include "debug_utils.h"
#include "eth_custom_phy_interface.h"

#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "stream_buffer.h"
#include "semphr.h"

#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "uart3_cmd_handler.h"

#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "plc_persist.h"
#include "plc_log.h"


// ===== UART RX/TX plumbing =====
#ifndef UART_STREAM_SIZE
#define UART_STREAM_SIZE        1024u
#endif

#ifndef UART_TRIGGER_LEVEL
#define UART_TRIGGER_LEVEL      1u
#endif

#ifndef UART_MB_SIZE
#define UART_MB_SIZE            1024u
#endif

#ifndef UART_STAGING_SZ
#define UART_STAGING_SZ         1024u
#endif

StreamBufferHandle_t g_uartRxStream;
MessageBufferHandle_t s_mb;
SemaphoreHandle_t s_txDone;

extern UART_HandleTypeDef huart3;
static uint8_t s_staging[UART_STAGING_SZ];

// ===== Threads/handles =====
static osThreadId_t thPlc;
static osThreadId_t thComms;
static osThreadId_t thUartTx;
static osThreadId_t thInit;
static osThreadId_t thDiag;

static TaskHandle_t hPlc;
static TaskHandle_t hComms;
static TaskHandle_t hUartTx;
static TaskHandle_t hInit;

// ===== Ethernet monitoring =====
extern struct netif gnetif;
static volatile uint32_t g_eth_evt = 0;

static const char *yesno(bool v) { return v ? "YES" : "NO"; }

static void eth_print_state(const char *reason) {
    char a[16], m[16], g[16];
    ip4addr_ntoa_r(netif_ip4_addr(&gnetif), a, sizeof(a));
    ip4addr_ntoa_r(netif_ip4_netmask(&gnetif), m, sizeof(m));
    ip4addr_ntoa_r(netif_ip4_gw(&gnetif), g, sizeof(g));

    int32_t phy = eth_phy_get_link_state();

    printf("[ETH] %s: netif_up=%s link_up=%s phy=%ld ip=%s mask=%s gw=%s\r\n",
           reason,
           yesno(netif_is_up(&gnetif)),
           yesno(netif_is_link_up(&gnetif)),
           (long) phy,
           a, m, g);
}

// ВАЖНО: callbacks только флаг — без printf (чтобы не раздувать EthLink stack)
static void netif_status_cb(struct netif *n) {
    (void) n;
    g_eth_evt = 1;
}

static void netif_link_cb(struct netif *n) {
    (void) n;
    g_eth_evt = 1;
}

static void EthMonTask(void *arg) {
    (void) arg;
    uint32_t last = 0;

    for (;;) {
//        uint32_t now = osKernelGetTickCount();
//        if (now - last >= 10000) {
//            last = now;
//            eth_print_state("tick");
//            if (g_eth_evt) {
//                g_eth_evt = 0;
//                eth_print_state("evt");
//            }
//        }
        osDelay(1000);
    }
}

static void MemReport_Print(void) {
    // Heap (FreeRTOS)
    uint32_t heapFree = (uint32_t) xPortGetFreeHeapSize();
    uint32_t heapMin = (uint32_t) xPortGetMinimumEverFreeHeapSize();
    uint32_t heapUsedMax = (uint32_t) (configTOTAL_HEAP_SIZE - heapMin);

    printf("[MEM] heap nowFree=%lu minEverFree=%lu heapUsedMax=%lu of %lu\r\n",
           (unsigned long) heapFree,
           (unsigned long) heapMin,
           (unsigned long) heapUsedMax,
           (unsigned long) configTOTAL_HEAP_SIZE);

    // Stacks (пример по твоим задачам)
    printf("[MEM] HWM free bytes: PLC=%lu COMMS=%lu UART=%lu INIT=%lu\r\n",
           (unsigned long) uxTaskGetStackHighWaterMark(hPlc) * sizeof(StackType_t),
           (unsigned long) uxTaskGetStackHighWaterMark(hComms) * sizeof(StackType_t),
           (unsigned long) uxTaskGetStackHighWaterMark(hUartTx) * sizeof(StackType_t),
           (unsigned long) uxTaskGetStackHighWaterMark(hInit) * sizeof(StackType_t));
}

// ===== Diagnostics (HWM/heap) =====
static void TaskDiag(void *arg) {
    (void) arg;

    for (;;) {
        MemReport_Print();
        printf("[HWM] PLC=%lu COMMS=%lu UART=%lu INIT=%lu heap=%lu min=%lu\r\n",
               (unsigned long)uxTaskGetStackHighWaterMark(hPlc)    * sizeof(StackType_t),
               (unsigned long)uxTaskGetStackHighWaterMark(hComms)  * sizeof(StackType_t),
               (unsigned long)uxTaskGetStackHighWaterMark(hUartTx) * sizeof(StackType_t),
               (unsigned long)uxTaskGetStackHighWaterMark(hInit)   * sizeof(StackType_t),
               (unsigned long)xPortGetFreeHeapSize(),
               (unsigned long)xPortGetMinimumEverFreeHeapSize());

        osDelay(10000);
    }
}

// ===== Tasks =====
void TaskInitial(void *arg) {
    (void) arg;

    MX_USB_DEVICE_Init();
    printf("&gnetif=%p\r\n", (void *) &gnetif);

    MX_LWIP_Init();

    // link/status callbacks (flag only)
    netif_set_status_callback(&gnetif, netif_status_cb);
    netif_set_link_callback(&gnetif, netif_link_cb);

    eth_print_state("after_lwip_init");

    // Start PLC TCP server
    PlcCommTcp_Start(5555);

    // Ethernet monitor task
    const osThreadAttr_t monAttr = {
            .name = "EthMon",
            .stack_size = 2048,      // bytes
            .priority = osPriorityLow
    };
    osThreadNew(EthMonTask, NULL, &monAttr);

    for (;;) {
        osDelay(10000);
    }
}

typedef struct {
    // counters per 1s window
    uint32_t cycles;
    uint32_t overruns;

    // plc_tick
    uint32_t tick_us_sum;
    uint32_t tick_us_max;

    // pump before scan
    uint32_t pre_cmd_sum;
    uint32_t pre_cmd_max;
    uint32_t pre_us_sum;
    uint32_t pre_us_max;

    // pump window (between scans)
    uint32_t win_cmd_sum;
    uint32_t win_cmd_max;
    uint32_t win_us_sum;
    uint32_t win_us_max;

} PlcProf1s;

static PlcProf1s s_prof;

static inline void prof_reset(void) { memset(&s_prof, 0, sizeof(s_prof)); }

static inline void prof_add_max(uint32_t *pMax, uint32_t v) { if (v > *pMax) *pMax = v; }

// call once per second
static void prof_log_and_reset(uint32_t period_ms, uint32_t heapFree, uint32_t heapMin) {
    if (s_prof.cycles == 0) return;

    uint32_t tick_avg = s_prof.tick_us_sum / s_prof.cycles;
    uint32_t pre_us_avg = s_prof.pre_us_sum / s_prof.cycles;
    uint32_t win_us_avg = s_prof.win_us_sum / s_prof.cycles;

    uint32_t pre_cmd_avg = s_prof.pre_cmd_sum / s_prof.cycles;
    uint32_t win_cmd_avg = s_prof.win_cmd_sum / s_prof.cycles;

    LOGT("[PLC.PROF] ",
         "period=%lums cycles=%lu overruns=%lu | "
         "tick avg=%luus max=%luus | "
         "prePump cmds avg=%lu max=%lu time avg=%luus max=%luus | "
         "winPump cmds avg=%lu max=%lu time avg=%luus max=%luus | "
         "heapFree=%lu heapMin=%lu\r\n",
         (unsigned long) period_ms,
         (unsigned long) s_prof.cycles,
         (unsigned long) s_prof.overruns,
         (unsigned long) tick_avg,
         (unsigned long) s_prof.tick_us_max,
         (unsigned long) pre_cmd_avg,
         (unsigned long) s_prof.pre_cmd_max,
         (unsigned long) pre_us_avg,
         (unsigned long) s_prof.pre_us_max,
         (unsigned long) win_cmd_avg,
         (unsigned long) s_prof.win_cmd_max,
         (unsigned long) win_us_avg,
         (unsigned long) s_prof.win_us_max,
         (unsigned long) heapFree,
         (unsigned long) heapMin
    );

    prof_reset();
}

// ---- DWT cycles -> microseconds (safe) ----
static inline uint32_t cycles_to_us(uint32_t cycles) {
    uint32_t hz = SystemCoreClock;
    if (hz == 0u) hz = 1u;
    // us = cycles * 1e6 / Hz (64-bit to avoid truncation)
    return (uint32_t) (((uint64_t) cycles * 1000000ull) / (uint64_t) hz);
}

static inline uint32_t dwt_now(void) { return DWT->CYCCNT; }

static inline uint32_t dwt_diff(uint32_t a, uint32_t b) {
    return (uint32_t) (b - a); // uint wrap ok
}

uint32_t lastOverrunLogMs = 0u;
uint32_t lastLongLogMs = 0u;

static inline int16_t clamp_u32_i16(uint32_t v) {
    return (v > 32767u) ? 32767 : (int16_t)v;
}

void TaskPlcScan(void *arg) {
    (void) arg;
    plc_log_init();

//    osDelay(10000);
    plc_log_put(PLC_EVT_LOG_VALUE, 100, (int16_t)(now_ms() & 0x7FFF));
    plc_log_put(PLC_EVT_LOG_VALUE, 101, (int16_t)(HAL_GetTick() & 0x7FFF));
    plc_log_put(PLC_EVT_LOG_VALUE, 102, (int16_t)(xTaskGetTickCount() & 0x7FFF));


    plc_mem_init();

    // ---- DWT init (Cortex-M7) ----
    SystemCoreClockUpdate();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    const uint32_t tickHz = osKernelGetTickFreq(); // ticks per second (обычно 100 или 1000)
    if (tickHz == 0u) {
        // защита от деления на 0 (не должно случаться)
        for (;;) { osDelay(1000); }
    }

    uint32_t nextTick = osKernelGetTickCount(); // ВНИМАНИЕ: ticks
    uint32_t prevPeriodMs = 10u;
    uint32_t prevCycleStartTick = 0u;

    uint32_t lastLogTick = osKernelGetTickCount(); // ticks
    prof_reset();

    // ---- tuning knobs ----
    const uint32_t GUARD_US_MIN = 200u;
    const uint32_t GUARD_US_MAX = 1000u;
    const uint32_t PRE_BUDGET_US = 300u;
    const uint32_t WIN_MAX_LOOPS = 16u;
    const uint32_t APPLY_MAX_ITEMS = 8u;

    for (;;) {
        // ---- cycle start stamps ----
        const uint32_t cycleStartTick = osKernelGetTickCount(); // ticks
        const uint32_t cycleStartCyc = DWT->CYCCNT;

        // ticks -> ms (монотонно)
        const uint32_t cycleStartMs = (uint32_t) ((uint64_t) cycleStartTick * 1000ull / (uint64_t) tickHz);

        // ---- real period between starts (в ms) ----
        if (prevCycleStartTick != 0u) {
            const uint32_t dtTicks = (uint32_t) (cycleStartTick - prevCycleStartTick);
            const uint32_t realCycleMs = (uint32_t) ((uint64_t) dtTicks * 1000ull / (uint64_t) tickHz);
            plc_metrics_on_cycle_ms(realCycleMs);
        }
        prevCycleStartTick = cycleStartTick;

        // ---- read cycle period (graph) in ms ----
        uint32_t periodMs;
        taskENTER_CRITICAL();
        periodMs = g_activeGraphValid ? g_activeGraph.cycleMs : 10u;
        taskEXIT_CRITICAL();

        if (periodMs == 0u) periodMs = 10u;
        if (periodMs < 5u) periodMs = 5u;
        if (periodMs > 1000u) periodMs = 1000u;

        // period in ticks (минимум 1 tick)
        uint32_t periodTicks = (uint32_t) ((uint64_t) periodMs * (uint64_t) tickHz / 1000ull);
        if (periodTicks == 0u) periodTicks = 1u;

        // ---- if period changed: resync schedule ----
        if (periodMs != prevPeriodMs) {
            nextTick = cycleStartTick;
            prevPeriodMs = periodMs;
        }

        const uint32_t periodUs = periodMs * 1000u;

        // Guard: 2% периода, но в диапазоне [GUARD_US_MIN..GUARD_US_MAX]
        uint32_t guardUs = periodUs / 50u;
        if (guardUs < GUARD_US_MIN) guardUs = GUARD_US_MIN;
        if (guardUs > GUARD_US_MAX) guardUs = GUARD_US_MAX;

        // =====================================================================
        // PRE-APPLY
        // =====================================================================
        {
            const uint32_t c0 = DWT->CYCCNT;
            const uint32_t pre_cmds = PlcMsg_PumpApply(APPLY_MAX_ITEMS, 0u, PRE_BUDGET_US);
            const uint32_t c1 = DWT->CYCCNT;

            const uint32_t pre_us = cycles_to_us((uint32_t) (c1 - c0));
            s_prof.pre_cmd_sum += pre_cmds;
            prof_add_max(&s_prof.pre_cmd_max, pre_cmds);
            s_prof.pre_us_sum += pre_us;
            prof_add_max(&s_prof.pre_us_max, pre_us);
        }

        // =====================================================================
        // SAFE-POINT: deferred load_active (flash operations outside PumpApply)
        // =====================================================================
        if (g_needLoadActive) {
            iwdg_feed();
            g_needLoadActive = false;
            // Реальная загрузка + подготовка staging + запрос активации
            // (plc_persist_load внутри уже делает upload->staging + plc_request_activate_graph)
            bool ok = plc_persist_load();
            plc_log_put(ok ? PLC_EVT_PERSIST_LOAD_OK : PLC_EVT_PERSIST_LOAD_FAIL, 0, 0);
            #if PLC_LOG_ENABLED
                        LOGT("PERSIST", "deferred load_active: %s", ok ? "OK" : "FAIL");
            #endif
        }

        // =====================================================================
        // PLC TICK (scan)  <-- ВАЖНО: передаём ms, НЕ ticks
        // =====================================================================
        const uint32_t tickStartCyc = DWT->CYCCNT;
        plc_tick(cycleStartMs);
        const uint32_t tickEndCyc = DWT->CYCCNT;

        const uint32_t tickUs = cycles_to_us((uint32_t) (tickEndCyc - tickStartCyc));
        s_prof.tick_us_sum += tickUs;
        prof_add_max(&s_prof.tick_us_max, tickUs);

        plc_metrics_on_cycle_us(tickUs, periodMs);

        // long tick (>=80% периода), тоже rate-limit 1/сек
        if (periodUs > 0u && tickUs >= (periodUs * 8u / 10u)) {
            if ((cycleStartMs - lastLongLogMs) >= 1000u) {
                lastLongLogMs = cycleStartMs;
                plc_log_put(PLC_EVT_SCAN_LONG,
                            clamp_u32_i16(tickUs),
                            (int16_t)periodMs);
            }
        }


        // =====================================================================
        // WINDOW APPLY (до дедлайна)
        // =====================================================================
        nextTick += periodTicks;  // дедлайн в ticks

        uint32_t win_cmds_total = 0u;
        uint32_t win_us_total = 0u;

        for (uint32_t loops = 0u; loops < WIN_MAX_LOOPS; loops++) {
            const uint32_t nowTick = osKernelGetTickCount();
            const int32_t leftTicks = (int32_t) (nextTick - nowTick);

            if (leftTicks <= 0) {
                s_prof.overruns++;

                // rate-limit: не чаще 1 раза/сек
                if ((cycleStartMs - lastOverrunLogMs) >= 1000u) {
                    lastOverrunLogMs = cycleStartMs;
                    plc_log_put(PLC_EVT_SCAN_OVERRUN,
                                clamp_u32_i16(tickUs),        // a = scanUs (lo/clamped)
                                (int16_t)periodMs);           // b = cycleMs
                }
                break;
            }


            // leftUs = leftTicks * 1_000_000 / tickHz
            uint32_t leftUs = (uint32_t) ((uint64_t) (uint32_t) leftTicks * 1000000ull / (uint64_t) tickHz);
            if (leftUs <= guardUs) break;

            uint32_t budgetUs = leftUs - guardUs;

            const uint32_t w0 = DWT->CYCCNT;
            const uint32_t done = PlcMsg_PumpApply(APPLY_MAX_ITEMS, 0u, budgetUs);
            const uint32_t w1 = DWT->CYCCNT;

            const uint32_t wUs = cycles_to_us((uint32_t) (w1 - w0));
            win_cmds_total += done;
            win_us_total += wUs;

            if (done == 0u) break;
        }

        s_prof.win_cmd_sum += win_cmds_total;
        prof_add_max(&s_prof.win_cmd_max, win_cmds_total);
        s_prof.win_us_sum += win_us_total;
        prof_add_max(&s_prof.win_us_max, win_us_total);

        s_prof.cycles++;

        // =====================================================================
        // WORK TIME
        // =====================================================================
        const uint32_t cycleEndCyc = DWT->CYCCNT;
        const uint32_t workUs = cycles_to_us((uint32_t) (cycleEndCyc - cycleStartCyc));
        plc_metrics_on_work_us(workUs);

        // =====================================================================
        // DEADLINE MISS / RESYNC (ticks)
        // =====================================================================
        const uint32_t endTick = osKernelGetTickCount();
        const int32_t overTicks = (int32_t) (endTick - nextTick);
        if (overTicks > 0) {
            s_prof.overruns++;
        }

        if ((int32_t) (nextTick - endTick) <= 0) {
            nextTick = endTick + periodTicks;
        }

        // =====================================================================
        // 10s log / prof reset (ticks)
        // =====================================================================
        const uint32_t tenSecTicks = 10u * tickHz;
        if ((int32_t) (endTick - lastLogTick) >= (int32_t) tenSecTicks) {
            lastLogTick = endTick;
            const uint32_t heapFree = (uint32_t) xPortGetFreeHeapSize();
            const uint32_t heapMin = (uint32_t) xPortGetMinimumEverFreeHeapSize();
            prof_log_and_reset(periodMs, heapFree, heapMin);
        }

        osDelayUntil(nextTick); // ожидает ticks
    }
}


void TaskComms(void *arg) {
    (void) arg;

    for (;;) {
        (void) Uart3_CommsProcessOnce(50); // ждём первый байт до 50мс, дальше дренаж без ожидания
//        size_t got = xStreamBufferReceive(g_uartRxStream, tmp, sizeof(tmp), portMAX_DELAY);
//        if (got > 0) {
//            process_bytes_in_task(tmp, got);
//        }
    }
}

// Единственный TX-таск (UART3 DMA)
void UartTx_Task(void *arg) {
    (void) arg;

    for (;;) {
        size_t n = xMessageBufferReceive(s_mb, s_staging, sizeof(s_staging), portMAX_DELAY);
        if (n == 0) continue;

        size_t off = 0;
        while (off < n) {

            size_t chunk = n - off;
            if (chunk > sizeof(s_staging)) chunk = sizeof(s_staging);

            // ВНИМАНИЕ: s_staging уже содержит данные [0..n),
            // поэтому для chunk < sizeof(staging) используем смещение.
            if (HAL_UART_Transmit_DMA(&huart3, (uint8_t *) (s_staging + off), (uint16_t) chunk) != HAL_OK) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }

            // Ждем завершение DMA
            xSemaphoreTake(s_txDone, portMAX_DELAY);
            off += chunk;
        }
    }
}

// ===== UART DMA TX complete callback -> give semaphore =====
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart == &huart3) {
//        BaseType_t hpw = pdFALSE;
//        xSemaphoreGiveFromISR(s_txDone, &hpw);
//        portYIELD_FROM_ISR(hpw);
//    }
//}

// ===== init =====
void init_rtos(void) {
    // FreeRTOS primitives
    g_uartRxStream = xStreamBufferCreate(UART_STREAM_SIZE, UART_TRIGGER_LEVEL);
    CHECK_OK(g_uartRxStream != NULL, "g_uartRxStream");

    s_mb = xMessageBufferCreate(UART_MB_SIZE);
    CHECK_OK(s_mb != NULL, "s_mb");

    s_txDone = xSemaphoreCreateBinary();
    CHECK_OK(s_txDone != NULL, "s_txDone");

    // CMSIS kernel
    CHECK_OK(osKernelInitialize() == osOK, "osKernelInitialize");

    // Single-writer CBOR queue (transport -> PLC)
    CHECK_OK(PlcMsg_Init(8), "PlcMsg_Init");

    const osThreadAttr_t plcAttr = {
            .name = "PLC",
            .stack_size = 8192,
            .priority = osPriorityRealtime
    };

//    const osThreadAttr_t commsAttr = {
//            .name = "COMMS",
//            .stack_size = 2048,
//            .priority = osPriorityAboveNormal
//    };

//    const osThreadAttr_t uartTxAttr = {
//            .name = "uart_tx",
//            .stack_size = 1024,
//            .priority = osPriorityAboveNormal
//    };

    const osThreadAttr_t initAttr = {
            .name = "Init",
            .stack_size = 2048,
            .priority = osPriorityNormal
    };

    const osThreadAttr_t diagAttr = {
            .name = "Diag",
            .stack_size = 2048,
            .priority = osPriorityLow
    };

    thPlc = osThreadNew(TaskPlcScan, NULL, &plcAttr);
    CHECK_OK(thPlc != NULL, "Task PLC");
    hPlc = (TaskHandle_t) thPlc;

//    thComms = osThreadNew(TaskComms, NULL, &commsAttr);
//    CHECK_OK(thComms != NULL, "Task COMMS");
//    hComms = (TaskHandle_t)thComms;
//
//    thUartTx = osThreadNew(UartTx_Task, NULL, &uartTxAttr);
//    CHECK_OK(thUartTx != NULL, "Task uart_tx");
//    hUartTx = (TaskHandle_t)thUartTx;

    thInit = osThreadNew(TaskInitial, NULL, &initAttr);
    CHECK_OK(thInit != NULL, "Task Init");
    hInit = (TaskHandle_t) thInit;

    thDiag = osThreadNew(TaskDiag, NULL, &diagAttr);
    CHECK_OK(thDiag != NULL, "Task Diag");

    // ---- AUTO-LOAD saved active graph on boot ----
    // Если нет сохранения - вернет false, это нормально.
//    plc_persist_clear();
    g_needLoadActive = true;


    CHECK_OK(osKernelStart() == osOK, "osKernelStart");

    for (;;) {}
}

// -------------------- TX helpers --------------------
BaseType_t UartTx_Send(const void *msg, size_t len, TickType_t to) {
    if (!s_mb || !msg || len == 0) return pdFALSE;

    size_t sent = xMessageBufferSend(s_mb, msg, len, to);
    return (sent == len) ? pdTRUE : pdFALSE;
}