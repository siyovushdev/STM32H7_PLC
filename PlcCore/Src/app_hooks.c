// app_hooks.c
#include "FreeRTOS.h"
#include "task.h"
#include "usb_log.h"
#include <stdatomic.h>

volatile atomic_int g_faultCode = 0;  // 1=malloc, 2=stack

void vApplicationMallocFailedHook(void) {
    g_faultCode = 1;
    UsbLog_printf("MALLOC FAILED: free=%u minEver=%u\r\n",
                  (unsigned)xPortGetFreeHeapSize(),
                  (unsigned)xPortGetMinimumEverFreeHeapSize());
    // НЕ отключаем прерывания сразу — дайте UsbLog_TX задаче выслать буфер
    // taskDISABLE_INTERRUPTS(); for(;;);
}

void vApplicationStackOverflowHook(TaskHandle_t t, char *name) {
    g_faultCode = 2;
    UsbLog_printf("STACK OVERFLOW: %s\r\n", name ? name : "?");
    // taskDISABLE_INTERRUPTS(); for(;;);
}

void vApplicationIdleHook(void) {
    static uint32_t last = 0;
    uint32_t now = xTaskGetTickCount();
    if ((now - last) > 1000) {         // раз в 1 с
        last = now;
        // Без блокировок: лучше выставить флаг,
        // а печать сделать в отдельной задачe-телеметрии.
    }
}
