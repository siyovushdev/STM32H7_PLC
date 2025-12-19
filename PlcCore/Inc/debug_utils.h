// debug_utils.h
#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "usb_log.h"
#include "logf.h"

#define CHECK_OK(expr, what) do{ \
    if(!(expr)){ \
        LOGT("CHECK_OK","ALLOC FAIL: %s free=%u minEver=%u\r\n", what, \
           xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize()); \
        configASSERT(0); \
    } \
}while(0)
