#include "cmsis_os2.h"

#include "plc.h"
#include "plc_comm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "message_buffer.h"
#include "semphr.h"

#include "debug_utils.h"
#include "usb_device.h"
#include "lwip.h"
#include "eth_custom_phy_interface.h"

#define UART_STREAM_SIZE     1024u
#define UART_TRIGGER_LEVEL   1u      // можно 1: пробуждать таск на любом байте

#ifndef UART_MB_SIZE
#define UART_MB_SIZE   1024  // кольцо сообщений в RAM
#endif
#ifndef UART_STAGING_SZ
#define UART_STAGING_SZ 1024  // линейный буфер под один DMA-транш
#endif

StreamBufferHandle_t g_uartRxStream;
MessageBufferHandle_t s_mb;
SemaphoreHandle_t s_txDone;
extern UART_HandleTypeDef huart3;

static uint8_t s_staging[UART_STAGING_SZ];

#define UART_STREAM_SIZE     1024u
#define UART_TRIGGER_LEVEL   1u

static osThreadId_t thPlc;
static osThreadId_t thComms;
static osThreadId_t thUartTx;
static osThreadId_t thInit;

void init_rtos(void) {
    // FreeRTOS buffers оставляем как есть (CMSIS их не заменяет)
    g_uartRxStream = xStreamBufferCreate(UART_STREAM_SIZE, UART_TRIGGER_LEVEL);
    CHECK_OK(g_uartRxStream != NULL, "g_uartRxStream");

    s_mb = xMessageBufferCreate(UART_MB_SIZE);
    CHECK_OK(s_mb != NULL, "s_mb");

    // Если s_txDone используется только в этом файле — переводи на CMSIS:
    // s_txDone = osSemaphoreNew(1, 0, NULL);
    // CHECK_OK(s_txDone != NULL, "s_txDone");

    // Инициализация CMSIS-RTOS2 kernel
    CHECK_OK(osKernelInitialize() == osOK, "osKernelInitialize");

    // Атрибуты потоков (stack_size — в БАЙТАХ)
    const osThreadAttr_t plcAttr = {
            .name = "PLC",
            .stack_size = 8192,
            .priority = osPriorityHigh
    };

    const osThreadAttr_t commsAttr = {
            .name = "COMMS",
            .stack_size = 8192,
            .priority = osPriorityAboveNormal
    };

    const osThreadAttr_t uartTxAttr = {
            .name = "uart_tx",
            .stack_size = 4096,
            .priority = osPriorityAboveNormal
    };

    const osThreadAttr_t initAttr = {
            .name = "initial",
            .stack_size = 4096,
            .priority = osPriorityNormal
    };

    thPlc = osThreadNew(TaskPlcScan, NULL, &plcAttr);
    CHECK_OK(thPlc != NULL, "Task PLC");

    thComms = osThreadNew(TaskComms, NULL, &commsAttr);
    CHECK_OK(thComms != NULL, "Task COMMS");

    thUartTx = osThreadNew(UartTx_Task, NULL, &uartTxAttr);
    CHECK_OK(thUartTx != NULL, "Task uart_tx");

    thInit = osThreadNew(TaskInitial, NULL, &initAttr);
    CHECK_OK(thInit != NULL, "Task initial");

    CHECK_OK(osKernelStart() == osOK, "osKernelStart");

    // сюда не должны возвращаться
    for (;;) {}
}

static void dbg_phy(void) {
    uint32_t id1 = 0, id2 = 0, bmsr = 0, pscsr = 0;
    // g_phyAddr у тебя статик внутри файла, поэтому для простоты прочитай через адрес 0..31 вручную
    for (uint32_t a = 0; a < 32; a++) {
        if (ETH_PHY_IO_ReadReg(a, 2, &id1) == 0 && ETH_PHY_IO_ReadReg(a, 3, &id2) == 0) {
            if (id1 != 0x0000 && id1 != 0xFFFF) {
                printf("PHY found addr=%lu id1=0x%04lx id2=0x%04lx\r\n", a, id1, id2);
                ETH_PHY_IO_ReadReg(a, 1, &bmsr);
                ETH_PHY_IO_ReadReg(a, 1, &bmsr); // второй раз (latched)
                ETH_PHY_IO_ReadReg(a, 0x1F, &pscsr);
                printf("BMSR=0x%04lx PSCSR=0x%04lx\r\n", bmsr, pscsr);
                break;
            }
        }
    }

    int32_t st = eth_phy_get_link_state();
    printf("eth_phy_get_link_state=%ld\r\n", (long) st);
}

extern struct netif gnetif;

static void dbg_ip(void) {
    char a[16], m[16], g[16];
    ip4addr_ntoa_r(netif_ip4_addr(&gnetif), a, sizeof(a));
    ip4addr_ntoa_r(netif_ip4_netmask(&gnetif), m, sizeof(m));
    ip4addr_ntoa_r(netif_ip4_gw(&gnetif), g, sizeof(g));
    printf("netif %s ip=%s mask=%s gw=%s\r\n",
           netif_is_link_up(&gnetif) ? "LINK_UP" : "LINK_DOWN",
           a, m, g);
}

static void dbg_mac(void) {
    printf("MAC=%02X:%02X:%02X:%02X:%02X:%02X\r\n",
           gnetif.hwaddr[0], gnetif.hwaddr[1], gnetif.hwaddr[2],
           gnetif.hwaddr[3], gnetif.hwaddr[4], gnetif.hwaddr[5]);
}

void TaskInitial(void *arg) {
    (void) arg;
    MX_USB_DEVICE_Init();
    printf("&gnetif=%p\r\n", (void *) &gnetif);
    MX_LWIP_Init();
    for (;;) {
        printf("pre\r\n");
        MX_LWIP_Process();
        printf("post\r\n");
        dbg_phy();
        MX_LWIP_Process();
        osDelay(1000);
    }
}

void TaskPlcScan(void *arg) {
    (void) arg;

    uint32_t last = osKernelGetTickCount();

    for (;;) {
        plc_tick();

        // период, например 10мс (подставь свой)
        const uint32_t period = 10;
        last += period;
        osDelayUntil(last);
    }
}


void TaskComms(void *arg) {
    uint8_t tmp[512];
    uint32_t counter = 0;
    for (;;) {
        LOGT("TaskComms", "[HWM] %-10s: stackFree=%u bytes, heapFree=%u, heapMin=%u\r\n",
             pcTaskGetName(NULL),
             (unsigned) uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t),
             (unsigned) xPortGetFreeHeapSize(),
             (unsigned) xPortGetMinimumEverFreeHeapSize()
        );
        size_t got = xStreamBufferReceive(g_uartRxStream, tmp, sizeof(tmp), portMAX_DELAY);
        if (got > 0) {
            process_bytes_in_task(tmp, got);
        }

//        if (++counter >= 1) {
//            counter = 0;
//
//            LOGT("TaskComms", "[HWM] %-10s: stackFree=%u bytes, heapFree=%u, heapMin=%u\r\n",
//                 pcTaskGetName(NULL),
//                 (unsigned)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t),
//                 (unsigned)xPortGetFreeHeapSize(),
//                 (unsigned)xPortGetMinimumEverFreeHeapSize()
//            );
//        }
    }
}

// Единственный TX-таск
void UartTx_Task(void *arg) {
    uint32_t counter = 0;
    for (;;) {
        // 1) Получаем одно сообщение целиком
        size_t n = xMessageBufferReceive(s_mb, s_staging, sizeof(s_staging), portMAX_DELAY);
        if (n == 0) continue;

        // 2) Отправляем через DMA; если сообщение длиннее staging — режем на куски
        size_t off = 0;
        while (off < n) {
            size_t chunk = n - off;
            if (chunk > sizeof(s_staging)) chunk = sizeof(s_staging);

            // Если кусок < staging (обычно так), у нас уже в буфере; если > — копируйте по месту по необходимости.
            // Здесь простой путь: отправляем имеющийся [off:off+chunk] напрямую.
            if (HAL_UART_Transmit_DMA(&huart3, (uint8_t *) ((&s_staging[0]) + off), chunk) != HAL_OK) {
                // Если UART занят/ошибка — можно подождать и повторить
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            // Ждём завершение DMA без блокировок других задач
            xSemaphoreTake(s_txDone, portMAX_DELAY);
            off += chunk;
        }

        if (++counter >= 1) {
            counter = 0;

            LOGT("TaskComms", "[HWM] %-10s: stackFree=%u bytes, heapFree=%u, heapMin=%u\r\n",
                 pcTaskGetName(NULL),
                 (unsigned) uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t),
                 (unsigned) xPortGetFreeHeapSize(),
                 (unsigned) xPortGetMinimumEverFreeHeapSize()
            );
        }
        // цикл вернётся за следующим сообщением
    }
}
