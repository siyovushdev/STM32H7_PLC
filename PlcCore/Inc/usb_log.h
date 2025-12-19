#pragma once
#include <stddef.h>
#include <stdint.h>

void UsbLog_Init(void);               // вызвать один раз (после MX_USB_DEVICE_Init)
void UsbLog_Put(const char* s, size_t n);
void UsbLog_Puts(const char* s);      // \0-строка
int  UsbLog_printf(const char* fmt, ...);
void UsbLog_PutFromISR(const char* s, size_t n);

void Usb_SendBlocking(const uint8_t* data, size_t n);
