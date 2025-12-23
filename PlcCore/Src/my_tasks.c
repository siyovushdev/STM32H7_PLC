#include "plc.h"
#include "plc_comm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "message_buffer.h"
#include "semphr.h"

#include "debug_utils.h"
#include "usb_device.h"

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

void init_rtos(void)
{
//    plcCmdQueue = xQueueCreate(8, sizeof(PlcCmd));
//    CHECK_OK(plcCmdQueue!=NULL, "plcCmdQueue 1K");

    g_uartRxStream = xStreamBufferCreate(UART_STREAM_SIZE, UART_TRIGGER_LEVEL);
    CHECK_OK(g_uartRxStream!=NULL, "g_uartRxStream 1K");

    s_mb = xMessageBufferCreate(UART_MB_SIZE);
    CHECK_OK(s_mb!=NULL, "s_mb 1K");

    s_txDone = xSemaphoreCreateBinary();
    CHECK_OK(s_txDone!=NULL, "s_txDone 1K");

    BaseType_t rc;
    rc = xTaskCreate(TaskPlcScan, "PLC",   1024, NULL, tskIDLE_PRIORITY+3, NULL);
    CHECK_OK(rc==pdPASS, "Task PLC");

    BaseType_t rc1;
    rc1 = xTaskCreate(TaskComms,   "COMMS", 2048, NULL, tskIDLE_PRIORITY+2, NULL);
    CHECK_OK(rc1==pdPASS, "Task COMMS");

    BaseType_t rc2;
    rc2 = xTaskCreate(UartTx_Task, "uart_tx", 1024, NULL, tskIDLE_PRIORITY+2, NULL);
    CHECK_OK(rc2==pdPASS, "Task uart_tx");

    vTaskStartScheduler();
}

void TaskPlcScan(void* arg)
{
    MX_USB_DEVICE_Init();
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t counter = 0;
    for (;;) {
        plc_tick();

        uint32_t periodMs =
                g_activeGraphValid ? g_activeGraph.cycleMs : 10u;

        if (++counter >= 500) {
            counter = 0;

            LOGT("TaskPlcScan", "[HWM] %-10s: stackFree=%u bytes, heapFree=%u, heapMin=%u\r\n",
                    pcTaskGetName(NULL),
                    (unsigned)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t),
                    (unsigned)xPortGetFreeHeapSize(),
                    (unsigned)xPortGetMinimumEverFreeHeapSize()
            );
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(periodMs));
    }
}


void TaskComms(void* arg)
{
    uint8_t tmp[512];
    uint32_t counter = 0;
    for (;;) {
        LOGT("TaskComms", "[HWM] %-10s: stackFree=%u bytes, heapFree=%u, heapMin=%u\r\n",
             pcTaskGetName(NULL),
             (unsigned)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t),
             (unsigned)xPortGetFreeHeapSize(),
             (unsigned)xPortGetMinimumEverFreeHeapSize()
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
void UartTx_Task(void *arg){
    uint32_t counter = 0;
    for(;;){
        // 1) Получаем одно сообщение целиком
        size_t n = xMessageBufferReceive(s_mb, s_staging, sizeof(s_staging), portMAX_DELAY);
        if (n == 0) continue;

        // 2) Отправляем через DMA; если сообщение длиннее staging — режем на куски
        size_t off = 0;
        while (off < n){
            size_t chunk = n - off;
            if (chunk > sizeof(s_staging)) chunk = sizeof(s_staging);

            // Если кусок < staging (обычно так), у нас уже в буфере; если > — копируйте по месту по необходимости.
            // Здесь простой путь: отправляем имеющийся [off:off+chunk] напрямую.
            if (HAL_UART_Transmit_DMA(&huart3, (uint8_t*)((&s_staging[0]) + off), chunk) != HAL_OK){
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
                 (unsigned)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t),
                 (unsigned)xPortGetFreeHeapSize(),
                 (unsigned)xPortGetMinimumEverFreeHeapSize()
            );
        }
        // цикл вернётся за следующим сообщением
    }
}
