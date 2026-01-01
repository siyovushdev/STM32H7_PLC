// plc_comm_tcp.c — robust TCP server (lwIP netconn) without per-frame malloc/free
// Protocol: [len_lo][len_hi][payload] where len = payload bytes (1..PLC_TCP_MAX_PAYLOAD)

#include "plc_comm_tcp.h"

#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "FreeRTOS.h"
#include "task.h"

#include "cbor_msg.h"   // CborReply + on_cbor_frame_received()
#include "plc_log.h"    // LOGT(TAG,...)
#include "logf.h"

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define TAG "PLC.TCP"

// ===== Config =====
#ifndef PLC_TCP_PORT_DEFAULT
#define PLC_TCP_PORT_DEFAULT          5555
#endif

#ifndef PLC_TCP_RECV_TIMEOUT_MS
#define PLC_TCP_RECV_TIMEOUT_MS       2000
#endif

#ifndef PLC_TCP_MAX_PAYLOAD
#define PLC_TCP_MAX_PAYLOAD           (64u * 1024u)
#endif

#ifndef PLC_TCP_MAX_FRAMES_PER_PUMP
#define PLC_TCP_MAX_FRAMES_PER_PUMP   8u
#endif

#ifndef PLC_TCP_RING_CAP
// Must be > (max_payload + 2) * 2 to tolerate fragmentation and chained pbufs
#define PLC_TCP_RING_CAP              ((PLC_TCP_MAX_PAYLOAD + 2u) * 2u + 256u)
#endif

// ===== Static buffers (one client at a time) =====
__attribute__((aligned(32)))
static uint8_t s_tcp_ring[PLC_TCP_RING_CAP];

__attribute__((aligned(32)))
static uint8_t s_payload_buf[PLC_TCP_MAX_PAYLOAD];

__attribute__((aligned(32)))
static uint8_t s_tx_frame[2u + PLC_TCP_MAX_PAYLOAD];

// ===== Ring buffer =====
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
    // keep 1 byte gap so r==w means empty
    return q->cap - ring_used(q) - 1u;
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

// memcpy-based write (fast)
static void ring_write(Ring *q, const uint8_t *src, uint32_t n)
{
    uint32_t cap = q->cap;
    uint32_t w = q->w;

    uint32_t first = cap - w;
    if (first > n) first = n;

    memcpy(&q->buf[w], src, first);
    w = (w + first) % cap;

    uint32_t left = n - first;
    if (left) {
        memcpy(&q->buf[w], src + first, left);
        w = (w + left) % cap;
    }

    q->w = w;
}

// memcpy-based read (fast)
static void ring_read(Ring *q, uint8_t *dst, uint32_t n)
{
    uint32_t cap = q->cap;
    uint32_t r = q->r;

    uint32_t first = cap - r;
    if (first > n) first = n;

    memcpy(dst, &q->buf[r], first);
    r = (r + first) % cap;

    uint32_t left = n - first;
    if (left) {
        memcpy(dst + first, &q->buf[r], left);
        r = (r + left) % cap;
    }

    q->r = r;
}

// ===== Send helpers =====
static err_t send_all(struct netconn *c, const uint8_t *data, uint16_t len)
{
#if LWIP_VERSION_MAJOR >= 2
    uint16_t sent = 0;
    while (sent < len) {
        size_t out = 0;
        err_t e = netconn_write_partly(
                c,
                data + sent,
                (size_t)(len - sent),
                NETCONN_COPY,
                &out
        );
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
    if (plen == 0 || plen > PLC_TCP_MAX_PAYLOAD) return ERR_VAL;

    s_tx_frame[0] = (uint8_t)(plen & 0xFFu);
    s_tx_frame[1] = (uint8_t)((plen >> 8) & 0xFFu);
    memcpy(&s_tx_frame[2], payload, plen);

    return send_all(c, s_tx_frame, (uint16_t)(2u + plen));
}

// ===== Frame parsing =====
static bool process_frames(struct netconn *c, Ring *q)
{
    for (uint32_t frames = 0; frames < PLC_TCP_MAX_FRAMES_PER_PUMP; frames++) {

        if (ring_used(q) < 2u) return true;

        uint16_t plen = (uint16_t)(ring_peek(q, 0) | ((uint16_t)ring_peek(q, 1) << 8));

        // resync protection
        if (plen == 0 || plen > PLC_TCP_MAX_PAYLOAD) {
            uint32_t u = ring_used(q);
            LOGT(TAG, "bad plen=%u used=%lu -> flush",
                 (unsigned)plen, (unsigned long)u);
            ring_drop(q, u);
            return true;
        }

        uint32_t need = 2u + (uint32_t)plen;
        if (ring_used(q) < need) return true;

        // drop header
        ring_drop(q, 2u);

        // read payload into static buffer
        ring_read(q, s_payload_buf, plen);

        // CBOR handler
        CborReply reply;
        reply.len = 0;
        on_cbor_frame_received(s_payload_buf, (size_t)plen, &reply);

        // reply
        if (reply.len > 0) {
            if (reply.len > PLC_TCP_MAX_PAYLOAD) {
                LOGT(TAG, "reply too big: %u", (unsigned)reply.len);
                continue;
            }
            err_t se = send_frame(c, reply.data, (uint16_t)reply.len);
            if (se != ERR_OK) {
                LOGT(TAG, "send_frame err=%d", (int)se);
                return false;
            }
        }
    }
    return true;
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
            if (nb) { netbuf_delete(nb); nb = NULL; }
            continue;
        }

        if (e != ERR_OK) {
            LOGT(TAG, "recv err=%d nb=%p -> disconnect", (int)e, nb);
            if (nb) { netbuf_delete(nb); nb = NULL; }
            break;
        }

        if (nb == NULL) {
            LOGT(TAG, "recv OK but nb==NULL -> disconnect");
            break;
        }

        // netbuf can be a chained pbuf list
        do {
            void *data = NULL;
            u16_t len = 0;
            netbuf_data(nb, &data, &len);

            if (len > 0 && data) {
                if (ring_free(&q) < (uint32_t)len) {
                    LOGT(TAG, "ring overflow free=%lu need=%u -> drop conn",
                         (unsigned long)ring_free(&q), (unsigned)len);
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

    err_t le = netconn_listen(s_srv);
    if (le != ERR_OK) {
        LOGT(TAG, "listen err=%d", (int)le);
        netconn_delete(s_srv);
        s_srv = NULL;
        s_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    LOGT(TAG, "listen port=%u", (unsigned)port);

    while (!s_stop) {
        struct netconn *c = NULL;
        err_t ae = netconn_accept(s_srv, &c);
        if (ae != ERR_OK || c == NULL) {
            LOGT(TAG, "accept err=%d c=%p", (int)ae, c);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        LOGT(TAG, "accept ok c=%p", c);

        // one client at a time
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

    // FreeRTOS stack is in WORDS, not bytes.
    // 4096 words = 16 KB. Adjust if needed.
    BaseType_t ok = xTaskCreate(
            tcp_server_task,
            "PlcTcpSrv",
            4096,
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
