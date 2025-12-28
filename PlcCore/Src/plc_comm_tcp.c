#include "plc_comm_tcp.h"

#include "lwip/api.h"
#include "lwip/sys.h"
#include "lwip/err.h"

#include "FreeRTOS.h"
#include "task.h"

#include "cbor_msg.h"   // CborReply + on_cbor_frame_received()
#include "plc_log.h"    // LOGT(tag,...)
#include "logf.h"

#include <string.h>
#include <stdio.h>

#define TAG "PLC.TCP"

// ===== Настройки =====
#ifndef PLC_TCP_PORT_DEFAULT
#define PLC_TCP_PORT_DEFAULT          5555
#endif

#ifndef PLC_TCP_RECV_TIMEOUT_MS
#define PLC_TCP_RECV_TIMEOUT_MS       2000
#endif

#ifndef PLC_TCP_MAX_PAYLOAD
// Подними до нужного тебе. Главное: хватит pbuf/mem в lwip и RAM/heap.
#define PLC_TCP_MAX_PAYLOAD           (64u * 1024u)
#endif

#ifndef PLC_TCP_RING_CAP
// Должно быть > (max_payload + 4) * 2, чтобы пережить большие netbuf/цепочки.
#define PLC_TCP_RING_CAP              ((PLC_TCP_MAX_PAYLOAD + 2u) * 2u + 256u)
#endif

// статический буфер (один клиент)
__attribute__((section(".tcp_buf"), aligned(32)))
static uint8_t s_tcp_ring[PLC_TCP_RING_CAP];

// ===== CRC16 (MODBUS) по [len_lo,len_hi] + payload =====
static uint16_t crc16_modbus_update(uint16_t crc, uint8_t in)
{
    crc ^= (uint16_t)in;
    for (uint8_t b = 0; b < 8; b++) {
        if (crc & 1u) crc = (uint16_t)((crc >> 1) ^ 0xA001u);
        else          crc >>= 1;
    }
    return crc;
}

static uint16_t crc16_modbus_len_payload(const uint8_t *payload, uint16_t len)
{
    uint16_t crc = 0xFFFFu;

    uint8_t hdr0 = (uint8_t)(len & 0xFFu);
    uint8_t hdr1 = (uint8_t)((len >> 8) & 0xFFu);

    crc = crc16_modbus_update(crc, hdr0);
    crc = crc16_modbus_update(crc, hdr1);

    for (uint16_t i = 0; i < len; i++) {
        crc = crc16_modbus_update(crc, payload[i]);
    }

    return crc;
}


// ===== Ring Buffer (поток TCP) =====
typedef struct {
    uint8_t *buf;
    uint32_t cap;
    uint32_t r;
    uint32_t w;
} Ring;

static void ring_init(Ring *q, uint8_t *storage, uint32_t cap)
{
    q->buf = storage;
    q->cap = cap;
    q->r = 0;
    q->w = 0;
}

static uint32_t ring_used(const Ring *q)
{
    return (q->w >= q->r) ? (q->w - q->r) : (q->cap - (q->r - q->w));
}

static uint32_t ring_free(const Ring *q)
{
    // оставляем 1 байт, чтобы r==w означало пусто
    return q->cap - ring_used(q) - 1u;
}

static void ring_write(Ring *q, const uint8_t *src, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        q->buf[q->w] = src[i];
        q->w = (q->w + 1u) % q->cap;
    }
}

static uint8_t ring_peek(const Ring *q, uint32_t off)
{
    uint32_t idx = (q->r + off) % q->cap;
    return q->buf[idx];
}

static void ring_drop(Ring *q, uint32_t n)
{
    q->r = (q->r + n) % q->cap;
}

static void ring_read(Ring *q, uint8_t *dst, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        dst[i] = q->buf[q->r];
        q->r = (q->r + 1u) % q->cap;
    }
}

// ===== Send helpers =====
static err_t send_all(struct netconn *c, const uint8_t *data, uint16_t len)
{
#if LWIP_VERSION_MAJOR >= 2
    uint16_t sent = 0;
    while (sent < len) {
        size_t out = 0;
        err_t e = netconn_write_partly(c, data + sent, (size_t)(len - sent), NETCONN_COPY, &out);
        if (e != ERR_OK) return e;
        if (out == 0) return ERR_CLSD;
        sent += (uint16_t)out;
    }
    return ERR_OK;
#else
    return netconn_write(c, data, len, NETCONN_COPY);
#endif
}

static err_t send_frame(struct netconn *c, const uint8_t *payload, uint16_t plen)
{
    // frame = [len_lo][len_hi][payload]
    uint32_t flen = 2u + (uint32_t)plen;

    uint8_t *frame = (uint8_t*)pvPortMalloc(flen);
    if (!frame) return ERR_MEM;

    frame[0] = (uint8_t)(plen & 0xFFu);
    frame[1] = (uint8_t)((plen >> 8) & 0xFFu);
    if (plen > 0) {
        memcpy(&frame[2], payload, plen);
    }

    err_t e = send_all(c, frame, (uint16_t)flen);
    vPortFree(frame);
    return e;
}


// ===== Frame parsing =====
static bool process_frames(struct netconn *c, Ring *q)
{
    while (1) {
        if (ring_used(q) < 2u) return true;

        uint16_t plen = (uint16_t)(ring_peek(q, 0) | ((uint16_t)ring_peek(q, 1) << 8));

        // защита от мусора/рассинхрона
        if (plen == 0 || plen > PLC_TCP_MAX_PAYLOAD) {
            ring_drop(q, 1);
            continue;
        }

        uint32_t need = 2u + (uint32_t)plen;
        if (ring_used(q) < need) return true;

        // снимаем len
        ring_drop(q, 2);

        uint8_t *payload = (uint8_t*)pvPortMalloc(plen);
        if (!payload) {
            LOGT(TAG, "NO HEAP for payload plen=%u", (unsigned)plen);
            // дропаем кадр целиком, чтобы не зависнуть
            ring_drop(q, plen);
            return false;
        }

        ring_read(q, payload, plen);

        // ===== CBOR обработка (твой существующий код) =====
        CborReply reply;
        reply.len = 0;
        on_cbor_frame_received(payload, (size_t)plen, &reply);
        vPortFree(payload);

        // ===== Ответ =====
        if (reply.len > 0) {
            if (reply.len > PLC_TCP_MAX_PAYLOAD) {
                LOGT(TAG, "reply too big: %u", (unsigned)reply.len);
                continue;
            }
            err_t e = send_frame(c, reply.data, (uint16_t)reply.len);
            if (e != ERR_OK) {
                LOGT(TAG, "send_frame err=%d", (int)e);
                return false; // оборвём соединение
            }
        }
    }
}

// ===== Client service =====
static void serve_client(struct netconn *c)
{
    netconn_set_recvtimeout(c, PLC_TCP_RECV_TIMEOUT_MS);

    Ring q;
    ring_init(&q, s_tcp_ring, PLC_TCP_RING_CAP);

    LOGT(TAG, "client connected");

    while (1) {
        struct netbuf *nb = NULL;
        err_t e = netconn_recv(c, &nb);

        if (e == ERR_TIMEOUT) {
            // таймаут – это норма, просто продолжаем
            continue;
        }

        if (e != ERR_OK || nb == NULL) {
            break; // клиент ушёл/ошибка
        }

        // netbuf может содержать цепочку pbuf — забираем все сегменты
        do {
            void *data = NULL;
            u16_t len = 0;
            netbuf_data(nb, &data, &len);

            if (len > 0) {
                if (ring_free(&q) < (uint32_t)len) {
                    LOGT(TAG, "ring overflow free=%u need=%u -> drop conn",
                         (unsigned)ring_free(&q), (unsigned)len);
                    netbuf_delete(nb);
                    goto exit;
                }
                ring_write(&q, (const uint8_t*)data, (uint32_t)len);

                if (!process_frames(c, &q)) {
                    netbuf_delete(nb);
                    goto exit;
                }
            }
        } while (netbuf_next(nb) >= 0);

        netbuf_delete(nb);
    }

    exit:
    LOGT(TAG, "client disconnected");
}

// ===== Server task =====
static TaskHandle_t s_task = NULL;
static struct netconn *s_srv = NULL;
static volatile bool s_stop = false;

static void tcp_server_task(void *arg)
{
    uint16_t port = (uint16_t)(uintptr_t)arg;
    if (port == 0) port = PLC_TCP_PORT_DEFAULT;

    s_stop = false;

    s_srv = netconn_new(NETCONN_TCP);
    if (!s_srv) {
        LOGT(TAG, "netconn_new failed");
        s_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    err_t e = netconn_bind(s_srv, IP_ADDR_ANY, port);
    if (e != ERR_OK) {
        LOGT(TAG, "bind err=%d", (int)e);
        netconn_delete(s_srv);
        s_srv = NULL;
        s_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    netconn_listen(s_srv);
    LOGT(TAG, "listen port=%u", (unsigned)port);

    while (!s_stop) {
        struct netconn *c = NULL;
        err_t ae = netconn_accept(s_srv, &c);
        if (ae != ERR_OK || c == NULL) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // Один клиент за раз (ПК). Для мультиклиента — spawn per connection.
        serve_client(c);

        netconn_close(c);
        netconn_delete(c);
    }

    if (s_srv) {
        netconn_close(s_srv);
        netconn_delete(s_srv);
        s_srv = NULL;
    }

    s_task = NULL;
    vTaskDelete(NULL);
}

bool PlcCommTcp_Start(uint16_t port)
{
    if (s_task) return true;

    BaseType_t ok = xTaskCreate(
            tcp_server_task,
            "PlcTcpSrv",
            4096,                          // подними, если мало
            (void*)(uintptr_t)port,
            tskIDLE_PRIORITY + 2,
            &s_task
    );

    if (ok != pdPASS) {
        s_task = NULL;
        return false;
    }
    return true;
}

void PlcCommTcp_Stop(void)
{
    s_stop = true;
}
