#include "usb_log.h"
#include "usbd_cdc_if.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#ifndef USB_LOG_BUF_SZ
#define USB_LOG_BUF_SZ 1024
#endif

static uint8_t ring[USB_LOG_BUF_SZ];
static volatile size_t w = 0, r = 0;
static osMutexId_t  mtx;
static osThreadId_t txTask;
#define USBLOG_FLAG (1u << 0)

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


static void UsbLog_Task(void* arg)
{
    (void)arg;
    uint8_t chunk[256];

    for (;;)
    {
        osThreadFlagsWait(USBLOG_FLAG, osFlagsWaitAny, 5);

        size_t n;
        do {
            n = 0;
            osMutexAcquire(mtx, osWaitForever);
            n = _pull(chunk, sizeof(chunk));
            osMutexRelease(mtx);

            if (n) {
                if (!cdc_ready()) {
                    continue;
                }
                uint32_t tries = 0;
                while (CDC_Transmit_FS(chunk, n) == USBD_BUSY) {
                    osDelay(1);
                    if (++tries > 20) break;
                }
            }
        } while (n);
    }
}


void UsbLog_Init(void)
{
    mtx = osMutexNew(NULL);
    const osThreadAttr_t a = {
            .name = "UsbLogTx",
            .stack_size = 4096,
            .priority = osPriorityNormal
    };
    txTask = osThreadNew(UsbLog_Task, NULL, &a);
}

void UsbLog_PutFromISR(const char* s, size_t n)
{
    if (!n) return;

    uint32_t prim = __get_PRIMASK();
    __disable_irq();

    size_t free = _space();
    if (n > free) n = free;
    _push((const uint8_t*)s, n);

    if (!prim) __enable_irq();

    if (txTask) {
        osThreadFlagsSet(txTask, USBLOG_FLAG);
    }
}


void UsbLog_Put(const char* s, size_t n) {
    if (!n) return;
    osMutexAcquire(mtx, osWaitForever);
    size_t free = _space();
    if (n > free) n = free;           // обрезаем тихо (или можно дожидаться)
    _push((const uint8_t*)s, n);
    osMutexRelease(mtx);
    // Разбудить TX-таску сразу, а не ждать 5 мс
    osThreadFlagsSet(txTask, USBLOG_FLAG);
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
