#include "plc_comm.h"
#include "plc.h"

// TinyCBOR
#include "cbor.h"  // поправь путь на свой tinycbor header
#include "cbor_msg.h"
#include "uart3_cmd_handler.h"
#include "plc_log.h"

#define FRAME_MAX_PAYLOAD  1024u

// FSM
static uint8_t  acc_buf[FRAME_MAX_PAYLOAD]; // храним только payload
static uint16_t acc_len = 0;                // накоплено в payload-буфере
static uint16_t acc_need = 0;               // сколько осталось дочитать (payload + CRC)
static uint8_t  len_lo = 0, len_hi = 0;
static bool     have_len_lo = false;
static bool     have_len_hi = false;

// временное хранение CRC байтов
static uint8_t  crc_lo = 0, crc_hi = 0;

static inline void rx_reset(void) {
    have_len_lo = false;
    have_len_hi = false;
    len_lo = 0;
    len_hi = 0;
    acc_len = 0;
    acc_need = 0;
    crc_lo = 0;
    crc_hi = 0;
}

// ---- protocol counters ----
static uint32_t g_rx_frames = 0;
static uint32_t g_rx_crc_err = 0;
static uint32_t g_rx_parse_err = 0;
static uint32_t g_tx_frames = 0;

void proto_count_rx_frame_ok(void){ g_rx_frames++; }
void proto_count_rx_crc_err(void){ g_rx_crc_err++; }
void proto_count_rx_parse_err(void){ g_rx_parse_err++; }
void proto_count_tx_frame(void){ g_tx_frames++; }

void proto_get_counters(uint32_t *rx_ok, uint32_t *rx_crc, uint32_t *rx_parse, uint32_t *tx_ok){
    if(rx_ok) *rx_ok = g_rx_frames;
    if(rx_crc) *rx_crc = g_rx_crc_err;
    if(rx_parse) *rx_parse = g_rx_parse_err;
    if(tx_ok) *tx_ok = g_tx_frames;
}

/* -----------------------------------------
 * CRC16 MODBUS (0xA001, init 0xFFFF)
 * Считаем по [LEN_LO LEN_HI] + payload
 * ----------------------------------------- */
uint16_t crc16_modbus(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;

    // сначала два байта длины (младший, затем старший)
    uint8_t hdr0 = (uint8_t)(len & 0xFF);
    uint8_t hdr1 = (uint8_t)(len >> 8);

    uint8_t in = hdr0;
    for (uint8_t b = 0; b < 8; b++) {
        uint16_t mix = (crc ^ in) & 1U;
        crc >>= 1;
        if (mix) crc ^= 0xA001U;
        in >>= 1;
    }
    in = hdr1;
    for (uint8_t b = 0; b < 8; b++) {
        uint16_t mix = (crc ^ in) & 1U;
        crc >>= 1;
        if (mix) crc ^= 0xA001U;
        in >>= 1;
    }

    // затем payload
    for (uint16_t i = 0; i < len; i++) {
        uint8_t x = data[i];
        for (uint8_t b = 0; b < 8; b++) {
            uint16_t mix = (crc ^ x) & 1U;
            crc >>= 1;
            if (mix) crc ^= 0xA001U;
            x >>= 1;
        }
    }
    return crc;
}

void process_bytes_in_task(const uint8_t *data, size_t n) {
    for (size_t i = 0; i < n; i++) {
        uint8_t b = data[i];

        if (!have_len_lo) {           // ждём len_lo
            len_lo = b;
            have_len_lo = true;
            continue;
        }
        if (!have_len_hi) {           // ждём len_hi
            len_hi = b;
            have_len_hi = true;

            uint16_t payloadLen = (uint16_t)(((uint16_t)len_hi << 8) | len_lo);
            // Важно: буфер хранит ТОЛЬКО payload; CRC хранится отдельно
            if (payloadLen == 0 || payloadLen > FRAME_MAX_PAYLOAD) {
                rx_reset();
                continue;
            }
            acc_len = 0;
            acc_need = (uint16_t)(payloadLen + 2); // payload + CRC(2)
            continue;
        }

        // читаем payload + CRC
        if (acc_need == 0) {
            // неожиданный байт — полный сброс
            rx_reset();
            continue;
        }

        uint16_t payloadLen = (uint16_t)(((uint16_t)len_hi << 8) | len_lo);

        if (acc_len < payloadLen) {
            // ещё собираем payload
            if (acc_len < FRAME_MAX_PAYLOAD) {
                acc_buf[acc_len++] = b;
            } else {
                // переполнение защиты (не должно случаться из-за проверки выше)
                rx_reset();
                continue;
            }
        } else if (acc_len == payloadLen) {
            // первый байт CRC (lo)
            crc_lo = b;
            acc_len++; // логический прогресс
        } else if (acc_len == payloadLen + 1) {
            // второй байт CRC (hi)
            crc_hi = b;
            acc_len++; // логический прогресс
        }
        acc_need--;

        // кадр собран
        if (acc_need == 0) {
            // посчитать CRC по [LEN+payload]
            uint16_t crc_calc = crc16_modbus(acc_buf, payloadLen);
            uint16_t crc_rx   = (uint16_t)(((uint16_t)crc_hi << 8) | crc_lo); // на проводе [lo][hi]

            if (crc_calc == crc_rx) {
                proto_count_rx_frame_ok();
                CborReply rep;
                on_cbor_frame_received(acc_buf, payloadLen, &rep);
                if (rep.len > 0) {

                    uart_send(rep.data, rep.len, pdMS_TO_TICKS(5));
                }
            } else{
                proto_count_rx_crc_err();
                plc_log_put(PLC_EVT_CRC_ERR, (int16_t)payloadLen, 0);
            }
            rx_reset(); // готов к следующему кадру
        }
    }
}



/* -----------------------------------------
 * UART TX: отправка кадра
 * ----------------------------------------- */
BaseType_t uart_send(const uint8_t *payload, uint16_t len, TickType_t to) {
    if (!payload || len > FRAME_MAX_PAYLOAD) return pdFALSE;

    const size_t flen = 2u + len + 2u; // LEN(2) + payload + CRC(2)

    // Малые кадры — со стека; крупные — из heap (FreeRTOS).
    uint8_t small[256];
    uint8_t *frame = (flen <= sizeof(small)) ? small
                                             : (uint8_t *) pvPortMalloc(flen);
    if (!frame) return pdFALSE;

    // LEN (little-endian)
    frame[0] = (uint8_t) (len & 0xFF);
    frame[1] = (uint8_t) (len >> 8);

    // PAYLOAD
    memcpy(&frame[2], payload, len);

    // CRC по payload
    const uint16_t crc = crc16_modbus(payload, len);
    frame[2 + len] = (uint8_t) (crc & 0xFF);
    frame[2 + len + 1] = (uint8_t) (crc >> 8);

    BaseType_t ok = UartTx_Send(frame, flen, to);

    if (frame != small) vPortFree(frame);
    return ok;
}
