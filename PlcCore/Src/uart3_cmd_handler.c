#include "uart3_cmd_handler.h"   // можешь переименовать в uart3_cmd_handler.h, если хочешь

#include "stream_buffer.h"
#include "message_buffer.h"
#include "cmsis_os2.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stddef.h>

#define FRAME_MAX_PAYLOAD  1024u

// -------------------- extern из твоего проекта --------------------
extern MessageBufferHandle_t s_mb;
extern osSemaphoreId_t s_txDone;        // бинарный семафор "TX DMA complete"
extern StreamBufferHandle_t  g_uartRxStream;

extern UART_HandleTypeDef    huart3;          // <-- ВАЖНО: теперь USART3

// -------------------- DMA RX буфер --------------------
// На H7 обязательно выравнивание под cache line (32 байта)
static uint8_t rx_buf[FRAME_MAX_PAYLOAD] __attribute__((aligned(32)));

// -------------------- D-Cache helpers --------------------
static inline uint32_t align32_down(uint32_t addr) { return addr & ~((uint32_t)31); }
static inline uint32_t align32_up(uint32_t addr)   { return (addr + 31u) & ~((uint32_t)31); }

static inline void dcache_invalidate(void *addr, uint32_t size)
{
    // Invalidate по cache line boundary
    uint32_t a = align32_down((uint32_t)addr);
    uint32_t e = align32_up((uint32_t)addr + size);
    SCB_InvalidateDCache_by_Addr((uint32_t*)a, (int32_t)(e - a));
}

// Если ты где-то стартуешь TX DMA с буфера в RAM — нужно clean.
// В ЭТОМ файле TX DMA не запускается напрямую, но на всякий случай оставлю.
static inline void dcache_clean(const void *addr, uint32_t size)
{
    uint32_t a = align32_down((uint32_t)addr);
    uint32_t e = align32_up((uint32_t)addr + size);
    SCB_CleanDCache_by_Addr((uint32_t*)a, (int32_t)(e - a));
}

// -------------------- RX start --------------------
void u3_start(void)
{
    // Запуск ReceiveToIdle DMA
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, FRAME_MAX_PAYLOAD);

    // Отключаем Half-Transfer IRQ (как у тебя было)
    if (huart3.hdmarx) {
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

// -------------------- TX helpers --------------------
BaseType_t UartTx_Send(const void *msg, size_t len, TickType_t to)
{
    if (!s_mb || !msg || len == 0) return pdFALSE;

    size_t sent = xMessageBufferSend(s_mb, msg, len, to);
    return (sent == len) ? pdTRUE : pdFALSE;
}

BaseType_t UartTx_SendFromISR(const void *msg, size_t len, BaseType_t *hpw)
{
    if (!s_mb || !msg || len == 0) return pdFALSE;

    size_t sent = xMessageBufferSendFromISR(s_mb, msg, len, hpw);
    return (sent == len) ? pdTRUE : pdFALSE;
}

// -------------------- HAL callbacks --------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart3) return;

    osSemaphoreRelease(s_txDone);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart3) goto rearm;
    if (size == 0) goto rearm;

    // На H7 с D-Cache нужно invalidate, иначе увидишь старые данные
    dcache_invalidate(rx_buf, size);

    BaseType_t xHPW = pdFALSE;
    (void)xStreamBufferSendFromISR(g_uartRxStream, rx_buf, size, &xHPW);
    portYIELD_FROM_ISR(xHPW);

    rearm:
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, FRAME_MAX_PAYLOAD);
    if (huart3.hdmarx) {
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}
