#pragma once
#include <stdint.h>
#include <string.h>

#ifndef CBOR_MAX_REPLY
#define CBOR_MAX_REPLY 1024u
#endif

typedef struct {
    uint8_t data[CBOR_MAX_REPLY];
    size_t  len;     // 0, если ответа нет
} CborReply;

/* Колбэк верхнего уровня: сюда падает ДЕКОДИРОВАННЫЙ CBOR-пакет */
void on_cbor_frame_received(const uint8_t* buf, size_t len, CborReply* out);