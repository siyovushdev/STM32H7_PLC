#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "main.h"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "stream_buffer.h"

/**
 * UART3 framing (legacy, same as your UART protocol):
 *   [len_lo][len_hi][payload...len bytes][crc_lo][crc_hi]
 * where CRC16 = Modbus over (len_lo,len_hi,payload...).
 *
 * Single-writer architecture:
 *  - UART transport task (TaskComms) MUST NOT touch PLC state.
 *  - It must push CBOR payload to PLC task via CborMsg_RpcCall().
 *  - PLC task executes command and returns reply payload (CBOR),
 *    which is sent back over UART in the same frame format.
 */

// Provided by your project (already exists)
BaseType_t UartTx_Send(const void *msg, size_t len, TickType_t to);              // from tasks
BaseType_t UartTx_SendFromISR(const void *msg, size_t len, BaseType_t *hpw);    // from ISR
void u3_start(void);                                                           // start RX DMA to IDLE etc.

/**
 * Call from TaskComms loop.
 * - waitMs: how long to wait for first byte (0 = poll).
 * Returns true if at least one full frame was processed.
 */
bool Uart3_CommsProcessOnce(uint32_t waitMs);

/**
 * Optional helpers if you need them from outside.
 */
uint32_t Uart3_GetRxDroppedFrames(void);
uint32_t Uart3_GetRxBadCrcFrames(void);
uint32_t Uart3_GetRxBadLenFrames(void);

#ifdef __cplusplus
}
#endif
