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

StreamBufferHandle_t  g_uartRxStream;
MessageBufferHandle_t s_mb;
SemaphoreHandle_t     s_txDone;

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

static const char* yesno(bool v) { return v ? "YES" : "NO"; }

static void eth_print_state(const char* reason)
{
    char a[16], m[16], g[16];
    ip4addr_ntoa_r(netif_ip4_addr(&gnetif), a, sizeof(a));
    ip4addr_ntoa_r(netif_ip4_netmask(&gnetif), m, sizeof(m));
    ip4addr_ntoa_r(netif_ip4_gw(&gnetif), g, sizeof(g));

    int32_t phy = eth_phy_get_link_state();

    printf("[ETH] %s: netif_up=%s link_up=%s phy=%ld ip=%s mask=%s gw=%s\r\n",
           reason,
           yesno(netif_is_up(&gnetif)),
           yesno(netif_is_link_up(&gnetif)),
           (long)phy,
           a, m, g);
}

// ВАЖНО: callbacks только флаг — без printf (чтобы не раздувать EthLink stack)
static void netif_status_cb(struct netif* n) { (void)n; g_eth_evt = 1; }
static void netif_link_cb  (struct netif* n) { (void)n; g_eth_evt = 1; }

static void EthMonTask(void* arg)
{
    (void)arg;
    uint32_t last = 0;

    for (;;)
    {
        uint32_t now = osKernelGetTickCount();
        if (now - last >= 1000) {
            last = now;
            eth_print_state("tick");
            if (g_eth_evt) {
                g_eth_evt = 0;
                eth_print_state("evt");
            }
        }
        osDelay(50);
    }
}

static void MemReport_Print(void)
{
    // Heap (FreeRTOS)
    uint32_t heapFree = (uint32_t)xPortGetFreeHeapSize();
    uint32_t heapMin  = (uint32_t)xPortGetMinimumEverFreeHeapSize();
    uint32_t heapUsedMax = (uint32_t)(configTOTAL_HEAP_SIZE - heapMin);

    printf("[MEM] heap nowFree=%lu minEverFree=%lu heapUsedMax=%lu of %lu\r\n",
           (unsigned long)heapFree,
           (unsigned long)heapMin,
           (unsigned long)heapUsedMax,
           (unsigned long)configTOTAL_HEAP_SIZE);

    // Stacks (пример по твоим задачам)
    printf("[MEM] HWM free bytes: PLC=%lu COMMS=%lu UART=%lu INIT=%lu\r\n",
           (unsigned long)uxTaskGetStackHighWaterMark(hPlc)    * sizeof(StackType_t),
           (unsigned long)uxTaskGetStackHighWaterMark(hComms)  * sizeof(StackType_t),
           (unsigned long)uxTaskGetStackHighWaterMark(hUartTx) * sizeof(StackType_t),
           (unsigned long)uxTaskGetStackHighWaterMark(hInit)   * sizeof(StackType_t));
}

// ===== Diagnostics (HWM/heap) =====
static void TaskDiag(void *arg)
{
    (void)arg;

    for (;;)
    {
        MemReport_Print();
        printf("[HWM] PLC=%lu COMMS=%lu UART=%lu INIT=%lu heap=%lu min=%lu\r\n",
               (unsigned long)uxTaskGetStackHighWaterMark(hPlc)    * sizeof(StackType_t),
               (unsigned long)uxTaskGetStackHighWaterMark(hComms)  * sizeof(StackType_t),
               (unsigned long)uxTaskGetStackHighWaterMark(hUartTx) * sizeof(StackType_t),
               (unsigned long)uxTaskGetStackHighWaterMark(hInit)   * sizeof(StackType_t),
               (unsigned long)xPortGetFreeHeapSize(),
               (unsigned long)xPortGetMinimumEverFreeHeapSize());

        osDelay(5000);
    }
}

// ===== Tasks =====
void TaskInitial(void *arg)
{
    (void)arg;

    MX_USB_DEVICE_Init();
    printf("&gnetif=%p\r\n", (void*)&gnetif);

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
        osDelay(1000);
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

static inline void prof_add_max(uint32_t* pMax, uint32_t v) { if (v > *pMax) *pMax = v; }

// call once per second
static void prof_log_and_reset(uint32_t period_ms, uint32_t heapFree, uint32_t heapMin)
{
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
         (unsigned long)period_ms,
         (unsigned long)s_prof.cycles,
         (unsigned long)s_prof.overruns,
         (unsigned long)tick_avg,
         (unsigned long)s_prof.tick_us_max,
         (unsigned long)pre_cmd_avg,
         (unsigned long)s_prof.pre_cmd_max,
         (unsigned long)pre_us_avg,
         (unsigned long)s_prof.pre_us_max,
         (unsigned long)win_cmd_avg,
         (unsigned long)s_prof.win_cmd_max,
         (unsigned long)win_us_avg,
         (unsigned long)s_prof.win_us_max,
         (unsigned long)heapFree,
         (unsigned long)heapMin
    );

    prof_reset();
}

void TaskPlcScan(void *arg)
{
    (void)arg;
    plc_mem_init();

    // ---- DWT init for microsecond profiling (Cortex-M7) ----
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t next = osKernelGetTickCount();
    uint32_t prevPeriod = 10u;

    uint32_t lastLogTick = osKernelGetTickCount();
    prof_reset();

    for (;;)
    {
        uint32_t period;
        taskENTER_CRITICAL();
        period = g_activeGraphValid ? g_activeGraph.cycleMs : 10u;
        taskEXIT_CRITICAL();

        if (period == 0) period = 10u;
        if (period < 5) period = 5;
        if (period > 1000) period = 1000;

        if (period != prevPeriod) {
            next = osKernelGetTickCount();
            prevPeriod = period;
        }

        // -------- prePump profile --------
        uint32_t c0 = DWT->CYCCNT;
        uint32_t pre_cmds = CborMsg_Pump(8u, 0u);
        uint32_t c1 = DWT->CYCCNT;
        uint32_t pre_us = (c1 - c0) / (SystemCoreClock / 1000000u);

        s_prof.pre_cmd_sum += pre_cmds;
        prof_add_max(&s_prof.pre_cmd_max, pre_cmds);
        s_prof.pre_us_sum += pre_us;
        prof_add_max(&s_prof.pre_us_max, pre_us);

        // -------- plc_tick profile --------
        uint32_t start = osKernelGetTickCount();

        uint32_t t0 = DWT->CYCCNT;
        plc_tick(start);
        uint32_t t1 = DWT->CYCCNT;

        uint32_t tick_us = (t1 - t0) / (SystemCoreClock / 1000000u);

        s_prof.tick_us_sum += tick_us;
        prof_add_max(&s_prof.tick_us_max, tick_us);

        uint32_t end = osKernelGetTickCount();
        plc_metrics_on_cycle(end - start, period);

        // -------- window pump profile --------
        next += period;

        uint32_t win_cmds_total = 0;
        uint32_t win_us_total = 0;

        for (;;) {
            uint32_t now = osKernelGetTickCount();
            if ((int32_t)(next - now) <= 0) {
                s_prof.overruns++;   // уже опоздали
                break;
            }

            uint32_t remain = next - now;

            uint32_t w0 = DWT->CYCCNT;
            uint32_t done = CborMsg_Pump(8u, remain);
            uint32_t w1 = DWT->CYCCNT;
            uint32_t w_us = (w1 - w0) / (SystemCoreClock / 1000000u);

            win_cmds_total += done;
            win_us_total += w_us;

            if (done == 0u) break; // timeout до next
        }

        s_prof.win_cmd_sum += win_cmds_total;
        prof_add_max(&s_prof.win_cmd_max, win_cmds_total);
        s_prof.win_us_sum += win_us_total;
        prof_add_max(&s_prof.win_us_max, win_us_total);

        s_prof.cycles++;

        // -------- 1s log --------
        uint32_t nowTick = osKernelGetTickCount();
        if ((int32_t)(nowTick - lastLogTick) >= 1000) {
            lastLogTick = nowTick;

            // если хочешь — подставь свои heap методы (FreeRTOS):
            uint32_t heapFree = (uint32_t)xPortGetFreeHeapSize();
            uint32_t heapMin  = (uint32_t)xPortGetMinimumEverFreeHeapSize();

            prof_log_and_reset(period, heapFree, heapMin);
        }

        osDelayUntil(next);
    }
}


void TaskComms(void *arg)
{
    (void)arg;

    for (;;)
    {
        (void)Uart3_CommsProcessOnce(50); // ждём первый байт до 50мс, дальше дренаж без ожидания
//        size_t got = xStreamBufferReceive(g_uartRxStream, tmp, sizeof(tmp), portMAX_DELAY);
//        if (got > 0) {
//            process_bytes_in_task(tmp, got);
//        }
    }
}

// Единственный TX-таск (UART3 DMA)
void UartTx_Task(void *arg)
{
    (void)arg;

    for (;;)
    {
        size_t n = xMessageBufferReceive(s_mb, s_staging, sizeof(s_staging), portMAX_DELAY);
        if (n == 0) continue;

        size_t off = 0;
        while (off < n) {

            size_t chunk = n - off;
            if (chunk > sizeof(s_staging)) chunk = sizeof(s_staging);

            // ВНИМАНИЕ: s_staging уже содержит данные [0..n),
            // поэтому для chunk < sizeof(staging) используем смещение.
            if (HAL_UART_Transmit_DMA(&huart3, (uint8_t*)(s_staging + off), (uint16_t)chunk) != HAL_OK) {
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
void init_rtos(void)
{
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
    CHECK_OK(CborMsg_InitQueue(16u), "CborMsg_InitQueue");

    const osThreadAttr_t plcAttr = {
            .name = "PLC",
            .stack_size = 4096,
            .priority = osPriorityHigh
    };

    const osThreadAttr_t commsAttr = {
            .name = "COMMS",
            .stack_size = 2048,
            .priority = osPriorityAboveNormal
    };

    const osThreadAttr_t uartTxAttr = {
            .name = "uart_tx",
            .stack_size = 1024,
            .priority = osPriorityAboveNormal
    };

    const osThreadAttr_t initAttr = {
            .name = "Init",
            .stack_size = 2048,
            .priority = osPriorityNormal
    };

    const osThreadAttr_t diagAttr = {
            .name = "Diag",
            .stack_size = 1028,
            .priority = osPriorityLow
    };

    thPlc   = osThreadNew(TaskPlcScan, NULL, &plcAttr);
    CHECK_OK(thPlc != NULL, "Task PLC");
    hPlc = (TaskHandle_t)thPlc;

    thComms = osThreadNew(TaskComms, NULL, &commsAttr);
    CHECK_OK(thComms != NULL, "Task COMMS");
    hComms = (TaskHandle_t)thComms;

    thUartTx = osThreadNew(UartTx_Task, NULL, &uartTxAttr);
    CHECK_OK(thUartTx != NULL, "Task uart_tx");
    hUartTx = (TaskHandle_t)thUartTx;

    thInit = osThreadNew(TaskInitial, NULL, &initAttr);
    CHECK_OK(thInit != NULL, "Task Init");
    hInit = (TaskHandle_t)thInit;

    thDiag = osThreadNew(TaskDiag, NULL, &diagAttr);
    CHECK_OK(thDiag != NULL, "Task Diag");

    CHECK_OK(osKernelStart() == osOK, "osKernelStart");

    for (;;) {}
}

// -------------------- TX helpers --------------------
BaseType_t UartTx_Send(const void *msg, size_t len, TickType_t to)
{
    if (!s_mb || !msg || len == 0) return pdFALSE;

    size_t sent = xMessageBufferSend(s_mb, msg, len, to);
    return (sent == len) ? pdTRUE : pdFALSE;
}