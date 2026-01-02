#include "uart3_cmd_handler.h"

#include "stm32h7xx_hal.h"

#include "cbor_msg.h"
#include "semphr.h"

#include <string.h>

// -------------------- Configuration --------------------
#ifndef UART3_FRAME_MAX_PAYLOAD
#define UART3_FRAME_MAX_PAYLOAD  1024u
#endif

#ifndef UART3_RPC_TIMEOUT_MS
#define UART3_RPC_TIMEOUT_MS     1500u
#endif

#ifndef UART3_TX_TIMEOUT_TICKS
#define UART3_TX_TIMEOUT_TICKS   pdMS_TO_TICKS(2000u)
#endif

// -------------------- Externs from your project --------------------
// RX stream where ISR pushes raw bytes (you said it already exists)
extern StreamBufferHandle_t  g_uartRxStream;
extern SemaphoreHandle_t     s_txDone;

// UART handle (CubeMX)
extern UART_HandleTypeDef huart3;

// Your RX DMA buffer (static) must exist somewhere; we keep our own to be self-contained.
static uint8_t s_rx_dma_buf[UART3_FRAME_MAX_PAYLOAD];

// -------------------- Small utilities --------------------
static uint16_t crc16_modbus(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 1u) crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            else          crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

// On STM32H7 D-Cache: invalidate aligned range before reading DMA-written memory.
// If you already have your own helper, you can replace this.
static void dcache_invalidate(void* addr, size_t len)
{
#if defined (SCB_InvalidateDCache_by_Addr)
    // Align addr down to 32-byte, len up to 32-byte
    uintptr_t a = (uintptr_t)addr;
    uintptr_t a32 = a & ~(uintptr_t)31u;
    uintptr_t end = (a + len + 31u) & ~(uintptr_t)31u;
    SCB_InvalidateDCache_by_Addr((uint32_t*)a32, (int32_t)(end - a32));
#else
    (void)addr; (void)len;
#endif
}

// -------------------- Stats --------------------
static uint32_t s_dropFrames = 0;
static uint32_t s_badCrcFrames = 0;
static uint32_t s_badLenFrames = 0;

uint32_t Uart3_GetRxDroppedFrames(void) { return s_dropFrames; }
uint32_t Uart3_GetRxBadCrcFrames(void)  { return s_badCrcFrames; }
uint32_t Uart3_GetRxBadLenFrames(void)  { return s_badLenFrames; }

// -------------------- RX ISR callback --------------------
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart3) return;

    // Invalidate DCache before touching DMA buffer
    dcache_invalidate(s_rx_dma_buf, size);

    BaseType_t hpw = pdFALSE;
    // Push bytes into stream buffer
    size_t sent = xStreamBufferSendFromISR(g_uartRxStream, s_rx_dma_buf, size, &hpw);
    if (sent != size) {
        // Stream full, drop
        s_dropFrames++;
    }
    portYIELD_FROM_ISR(hpw);

    // Rearm RX to IDLE DMA
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_rx_dma_buf, UART3_FRAME_MAX_PAYLOAD);
    if (huart3.hdmarx) {
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT); // no half-transfer irq
    }
}

void u3_start(void)
{
    // Start RX DMA to IDLE
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, s_rx_dma_buf, UART3_FRAME_MAX_PAYLOAD);
    if (huart3.hdmarx) {
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

// -------------------- Frame assembler (runs in TaskComms) --------------------
typedef enum {
    ST_LEN0 = 0,
    ST_LEN1,
    ST_PAYLOAD,
    ST_CRC0,
    ST_CRC1,
} RxState;

static RxState s_rxState = ST_LEN0;
static uint16_t s_rxLen = 0;
static uint16_t s_rxCrc = 0;
static uint16_t s_calcCrc = 0;

static uint8_t  s_payload[UART3_FRAME_MAX_PAYLOAD];
static size_t   s_payloadPos = 0;

static uint8_t  s_txFrame[2u + UART3_FRAME_MAX_PAYLOAD + 2u]; // len + payload + crc
static uint8_t  s_crcBuf[2u + UART3_FRAME_MAX_PAYLOAD];       // for crc calculation

static void rx_reset(void)
{
    s_rxState = ST_LEN0;
    s_rxLen = 0;
    s_rxCrc = 0;
    s_calcCrc = 0;
    s_payloadPos = 0;
}

static void send_reply_frame(const CborReply* reply)
{
    if (!reply || reply->len == 0) return;

    if (reply->len > UART3_FRAME_MAX_PAYLOAD) {
        // Too big for UART legacy framing: drop.
        // (If you need bigger over UART: increase UART3_FRAME_MAX_PAYLOAD and DMA buffer.)
        return;
    }

    uint16_t len = (uint16_t)reply->len;
    s_txFrame[0] = (uint8_t)(len & 0xFFu);
    s_txFrame[1] = (uint8_t)((len >> 8) & 0xFFu);
    memcpy(&s_txFrame[2], reply->data, reply->len);

    // crc over len + payload
    s_crcBuf[0] = s_txFrame[0];
    s_crcBuf[1] = s_txFrame[1];
    memcpy(&s_crcBuf[2], reply->data, reply->len);
    uint16_t crc = crc16_modbus(s_crcBuf, (size_t)2u + reply->len);

    s_txFrame[2u + reply->len + 0u] = (uint8_t)(crc & 0xFFu);
    s_txFrame[2u + reply->len + 1u] = (uint8_t)((crc >> 8) & 0xFFu);

    (void)UartTx_Send(s_txFrame, (size_t)2u + reply->len + 2u, UART3_TX_TIMEOUT_TICKS);
}

bool Uart3_CommsProcessOnce(uint32_t waitMs)
{
    // We read raw bytes from the stream buffer and assemble frames.
    // waitMs applies ONLY for first byte, then we drain without waiting.
    uint8_t b;
    size_t got = xStreamBufferReceive(g_uartRxStream, &b, 1u, pdMS_TO_TICKS(waitMs));
    if (got != 1u) return false;

    bool processedAny = false;

    for (;;) {
        switch (s_rxState) {
            case ST_LEN0:
                s_rxLen = (uint16_t)b;
                s_rxState = ST_LEN1;
                break;

            case ST_LEN1:
                s_rxLen |= (uint16_t)((uint16_t)b << 8);
                if (s_rxLen == 0 || s_rxLen > UART3_FRAME_MAX_PAYLOAD) {
                    s_badLenFrames++;
                    rx_reset();
                } else {
                    s_payloadPos = 0;
                    s_rxState = ST_PAYLOAD;
                }
                break;

            case ST_PAYLOAD:
                s_payload[s_payloadPos++] = b;
                if (s_payloadPos >= s_rxLen) {
                    s_rxState = ST_CRC0;
                }
                break;

            case ST_CRC0:
                s_rxCrc = (uint16_t)b;
                s_rxState = ST_CRC1;
                break;

            case ST_CRC1:
                s_rxCrc |= (uint16_t)((uint16_t)b << 8);

                // calc crc over len + payload
                s_crcBuf[0] = (uint8_t)(s_rxLen & 0xFFu);
                s_crcBuf[1] = (uint8_t)((s_rxLen >> 8) & 0xFFu);
                memcpy(&s_crcBuf[2], s_payload, s_rxLen);
                s_calcCrc = crc16_modbus(s_crcBuf, (size_t)2u + s_rxLen);

                if (s_calcCrc != s_rxCrc) {
                    s_badCrcFrames++;
                    rx_reset();
                    break;
                }

                // ✅ Single-writer: execute command in PLC task via queue/RPC
                CborReply reply;
                memset(&reply, 0, sizeof(reply));
                (void)CborMsg_RpcCall(s_payload, (size_t)s_rxLen, &reply, UART3_RPC_TIMEOUT_MS);

                // send reply over UART (legacy framed)
                send_reply_frame(&reply);

                processedAny = true;
                rx_reset();
                break;

            default:
                rx_reset();
                break;
        }

        // Drain remaining bytes without waiting
        got = xStreamBufferReceive(g_uartRxStream, &b, 1u, 0u);
        if (got != 1u) break;
    }

    return processedAny;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3) {
        BaseType_t hpw = pdFALSE;
        xSemaphoreGiveFromISR(s_txDone, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}
