#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CBOR_MAX_REPLY
#define CBOR_MAX_REPLY 8192u
#endif

typedef struct {
    uint8_t data[CBOR_MAX_REPLY];
    size_t  len;     // 0, если ответа нет
} CborReply;

/**
 * Инициализация очереди single-writer CBOR команд.
 * Вызывать ПОСЛЕ osKernelInitialize() и ДО osKernelStart().
 */
bool CborMsg_InitQueue(uint32_t depth);

/**
 * RPC-вызов: transport-поток (TCP/UART) передаёт фрейм PLC-таску и ждёт ответ.
 * frame: CBOR payload (без [len_lo][len_hi])
 * timeoutMs: таймаут ожидания обработки PLC-таском.
 * Возвращает true, если команда поставлена в очередь и дождались завершения (даже если reply.len==0).
 */
bool CborMsg_RpcCall(const uint8_t* frame, size_t len, CborReply* reply, uint32_t timeoutMs);

/**
 * Pump очереди CBOR-команд. Вызывать ТОЛЬКО из PLC task (single-writer).
 * maxItems: максимум команд за один вызов.
 * timeoutMsFirst: сколько ждать первую команду (0 = не ждать). После первой команды остальные забираются без ожидания.
 * Возвращает количество обработанных команд.
 */
uint32_t CborMsg_Pump(uint32_t maxItems, uint32_t timeoutMsFirst);

/* Старый entrypoint оставлен для совместимости, но теперь его должен вызывать только PLC task. */
void on_cbor_frame_received(const uint8_t* buf, size_t len, CborReply* out);

#ifdef __cplusplus
}
#endif
