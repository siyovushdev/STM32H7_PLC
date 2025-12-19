#include "usb_log.h"
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#ifndef USB_LOG_BUF_SZ
#define USB_LOG_BUF_SZ 1024
#endif

static uint8_t ring[USB_LOG_BUF_SZ];
static volatile size_t w = 0, r = 0;
static SemaphoreHandle_t mtx;
static TaskHandle_t txTask;

static inline int cdc_ready(void){
    extern USBD_HandleTypeDef hUsbDeviceFS;
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}

static size_t _space(void) {
    size_t rw = w, rr = r;
    return (rr <= rw) ? (USB_LOG_BUF_SZ - (rw - rr) - 1)
                      : (rr - rw - 1);
}
static void _push(const uint8_t* d, size_t n) {
    while (n--) {
        ring[w] = *d++;
        w = (w + 1) % USB_LOG_BUF_SZ;
    }
}
static size_t _pull(uint8_t* d, size_t maxn) {
    size_t cnt = 0;
    while ((r != w) && (cnt < maxn)) {
        *d++ = ring[r];
        r = (r + 1) % USB_LOG_BUF_SZ;
        cnt++;
    }
    return cnt;
}


void Usb_SendBlocking(const uint8_t* data, size_t n)
{
    while (n > 0) {
        uint16_t chunk = (n > 64) ? 64 : (uint16_t)n;  // размер пакета USB FS

        uint32_t t0 = HAL_GetTick();
        while (CDC_Transmit_FS((uint8_t*)data, chunk) == USBD_BUSY) {
            if ((HAL_GetTick() - t0) > 20) {
                // более 20 мс BUSY — дропаем остаток
                return;
            }
        }

        data += chunk;
        n    -= chunk;
    }
}


static void UsbLog_Task(void* arg) {
    uint8_t chunk[256];
    for (;;) {
        // ждём уведомления или таймаут
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        size_t n;
        do {
            n = 0;
            xSemaphoreTake(mtx, portMAX_DELAY);
            n = _pull(chunk, sizeof(chunk));
            xSemaphoreGive(mtx);
            if (n) {
                if (!cdc_ready()) {
                    // USB не готов — просто дропаем кусок (или положите обратно, если нужно)
                    continue;
                }
                // Ограничим «залипание» на BUSY, чтобы не блокировать систему
                uint32_t tries = 0;
                while (CDC_Transmit_FS(chunk, n) == USBD_BUSY) {
                    vTaskDelay(pdMS_TO_TICKS(1));
                    if (++tries > 20) { // ~20 мс терпения
                        break;          // дропаем остаток, идём дальше
                    }
                }
            }
        } while (n);
    }
}

void UsbLog_Init(void) {
    mtx = xSemaphoreCreateMutex();
    xTaskCreate(UsbLog_Task, "UsbLogTx", 1024, NULL, tskIDLE_PRIORITY+2, &txTask);
}

void UsbLog_PutFromISR(const char* s, size_t n) {
    if (!n) return;
    UBaseType_t prim = taskENTER_CRITICAL_FROM_ISR();
    size_t free = _space();
    if (n > free) n = free;               // без ожиданий — лишнее отбрасываем
    _push((const uint8_t*)s, n);
    taskEXIT_CRITICAL_FROM_ISR(prim);
    if (txTask) {                    // ВАЖНО: проверяем handle
        vTaskNotifyGiveFromISR(txTask, NULL); // разбудить TX-таску
    }
}

void UsbLog_Put(const char* s, size_t n) {
    if (!n) return;
    xSemaphoreTake(mtx, portMAX_DELAY);
    size_t free = _space();
    if (n > free) n = free;           // обрезаем тихо (или можно дожидаться)
    _push((const uint8_t*)s, n);
    xSemaphoreGive(mtx);
    // Разбудить TX-таску сразу, а не ждать 5 мс
    xTaskNotifyGive(txTask);
}

void UsbLog_Puts(const char* s) {
    UsbLog_Put(s, strlen(s));
}

int UsbLog_printf(const char* fmt, ...) {
    char buf[256];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return n;
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    UsbLog_Put(buf, (size_t)n);
    return n;
}
