#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "portmacro.h"

/*
* Структура кадра, который мы принимаем из UART.
* Формат физически такой:
*   [len_lo][len_hi][payload...len байт...][crc_lo][crc_hi]
*
* payload = CBOR-карта вида:
*   {"cmd":"status"} или
*   {"cmd":"upload_graph","graph":{...}} и т.д.
*/

//typedef struct {
//    uint16_t len;
//    uint8_t  payload[2048]; // лимит задаёшь сам
//} Frame;

/* CRC16 (MODBUS-style) */
uint16_t crc16_modbus(const uint8_t* data, uint16_t len);


BaseType_t uart_send(const uint8_t* payload, uint16_t len, TickType_t to);

void process_bytes_in_task(const uint8_t* data, size_t n);


#ifdef __cplusplus
}
#endif