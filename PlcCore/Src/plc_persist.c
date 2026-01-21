#include "plc_persist.h"

#include "main.h"
#include "stm32h7xx_hal.h"

#include "plc.h"
#include "logf.h"

#include <string.h>
#include <stdint.h>
#include <stddef.h>   // offsetof

#define TAG "PERSIST"

#define PLC_PERSIST_MAGIC     (0x504C4350u)  // 'PLCP'
#define PLC_PERSIST_VERSION   (1u)

#define H7_SECTOR_SIZE        (0x20000u)     // 128KB typical for H743
#define FLASHWORD_SIZE        (32u)          // 256-bit = 32 bytes

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t data_len;     // bytes
    uint32_t crc32;        // optional, can be 0
    uint32_t reserved[4];
} PlcPersistHeader;

// =====================
// CRC32 (простая, достаточно для контроля целостности)
// =====================
static uint32_t crc32_simple(const void *data, uint32_t len) {
    const uint8_t *p = (const uint8_t *) data;
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (uint32_t b = 0; b < 8; b++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

// =====================
// Bank/Sector helpers
// =====================
static inline uint32_t addr_to_bank(uint32_t addr) {
    return (addr < FLASH_BANK2_BASE) ? FLASH_BANK_1 : FLASH_BANK_2;
}

static inline uint32_t bank_base(uint32_t bank) {
    return (bank == FLASH_BANK_1) ? FLASH_BANK1_BASE : FLASH_BANK2_BASE;
}

static inline uint32_t addr_to_sector(uint32_t addr) {
    uint32_t bank = addr_to_bank(addr);
    uint32_t base = bank_base(bank);
    return (addr - base) / H7_SECTOR_SIZE;
}

static void clear_flash_flags(uint32_t bank) {
    if (bank == FLASH_BANK_1) {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_EOP_BANK1);
    } else {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2 | FLASH_FLAG_EOP_BANK2);
    }
}

static void log_flash_error(void) {
    uint32_t err = HAL_FLASH_GetError();
    LOGT(TAG, "HAL_FLASH_GetError=0x%08lX", (unsigned long) err);
}

// =====================
// Flash erase / program
// =====================
static HAL_StatusTypeDef flash_erase_sector(uint32_t addr) {
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sectorError = 0;

    uint32_t bank = addr_to_bank(addr);
    uint32_t sector = addr_to_sector(addr);

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = bank;
    erase.Sector = sector;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();
    clear_flash_flags(bank);

    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &sectorError);
    if (st != HAL_OK) {
        LOGT(TAG, "Erase FAIL: addr=0x%08lX bank=%lu sector=%lu sectorError=%lu",
             (unsigned long) addr,
             (unsigned long) bank,
             (unsigned long) sector,
             (unsigned long) sectorError);
        log_flash_error();
    }

    HAL_FLASH_Lock();
    return st;
}

// Пишем потоково по 32 байта (FLASHWORD), без огромных буферов.
static HAL_StatusTypeDef flash_write_stream(uint32_t addr, const PlcPersistHeader *h, const void *data, uint32_t len) {
    if ((addr % FLASHWORD_SIZE) != 0u) {
        LOGT(TAG, "Write FAIL: addr not 32B aligned: 0x%08lX", (unsigned long) addr);
        return HAL_ERROR;
    }

    const uint8_t *hdr = (const uint8_t *)h;
    const uint8_t *payload = (const uint8_t *)data;

    const uint32_t total = (uint32_t)sizeof(PlcPersistHeader) + len;
    const uint32_t padded = (total + (FLASHWORD_SIZE - 1u)) & ~(FLASHWORD_SIZE - 1u);

    if (padded > PLC_PERSIST_SIZE) {
        LOGT(TAG, "Write FAIL: padded=%lu > sector=%lu",
             (unsigned long)padded, (unsigned long)PLC_PERSIST_SIZE);
        return HAL_ERROR;
    }

    uint32_t bank = addr_to_bank(addr);

    HAL_FLASH_Unlock();
    clear_flash_flags(bank);

    HAL_StatusTypeDef st = HAL_OK;

    for (uint32_t off = 0; off < padded; off += FLASHWORD_SIZE) {
        uint8_t block[FLASHWORD_SIZE] __attribute__((aligned(32)));
        memset(block, 0xFF, sizeof(block));

        for (uint32_t j = 0; j < FLASHWORD_SIZE; j++) {
            uint32_t idx = off + j;

            if (idx < (uint32_t)sizeof(PlcPersistHeader)) {
                block[j] = hdr[idx];
            } else if (idx < total) {
                block[j] = payload[idx - (uint32_t)sizeof(PlcPersistHeader)];
            } else {
                // padding stays 0xFF
            }
        }

        uint32_t fw[8] __attribute__((aligned(32)));
        memcpy(fw, block, FLASHWORD_SIZE);

        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr + off, (uint32_t)fw);
        if (st != HAL_OK) {
            LOGT(TAG, "Program FAIL: addr=0x%08lX off=%lu", (unsigned long)(addr + off), (unsigned long)off);
            log_flash_error();
            break;
        }
    }

    HAL_FLASH_Lock();

#if (__DCACHE_PRESENT == 1U)
    // invalidate cache lines covering written flash range
    SCB_InvalidateDCache_by_Addr((uint32_t *)addr, (int32_t)padded);
#endif
    __DSB();
    __ISB();

    return st;
}

// =====================
// Low-level blob API
// =====================
static inline const uint8_t *flash_ptr(uint32_t addr) {
    return (const uint8_t *)(uintptr_t)addr;
}

bool plc_persist_load_blob(void *out_buf, uint32_t out_max, uint32_t *out_len) {
    if (!out_buf || out_max == 0u) return false;

    const PlcPersistHeader *h = (const PlcPersistHeader *)flash_ptr(PLC_PERSIST_BASE);

    if (h->magic != PLC_PERSIST_MAGIC) {
        LOGT(TAG, "No persist: bad magic 0x%08lX", (unsigned long)h->magic);
        return false;
    }
    if (h->version != PLC_PERSIST_VERSION) {
        LOGT(TAG, "No persist: version %lu != %lu",
             (unsigned long)h->version, (unsigned long)PLC_PERSIST_VERSION);
        return false;
    }
    if (h->data_len == 0u || h->data_len > PLC_PERSIST_MAX_DATA || h->data_len > out_max) {
        LOGT(TAG, "No persist: bad len=%lu", (unsigned long)h->data_len);
        return false;
    }

    const uint8_t *data = flash_ptr(PLC_PERSIST_BASE + sizeof(PlcPersistHeader));
    uint32_t crc = crc32_simple(data, h->data_len);
    if (h->crc32 != 0u && crc != h->crc32) {
        LOGT(TAG, "No persist: CRC mismatch calc=0x%08lX stored=0x%08lX",
             (unsigned long)crc, (unsigned long)h->crc32);
        return false;
    }

    memcpy(out_buf, data, h->data_len);
    if (out_len) *out_len = h->data_len;

    LOGT(TAG, "Persist blob loaded: len=%lu", (unsigned long)h->data_len);
    return true;
}

bool plc_persist_save_blob(const void *data, uint32_t len) {
    if (!data || len == 0u || len > PLC_PERSIST_MAX_DATA) {
        LOGT(TAG, "Persist blob save: bad args len=%lu", (unsigned long)len);
        return false;
    }

    PlcPersistHeader h;
    memset(&h, 0, sizeof(h));
    h.magic = PLC_PERSIST_MAGIC;
    h.version = PLC_PERSIST_VERSION;
    h.data_len = len;
    h.crc32 = crc32_simple(data, len);

    const uint32_t total = (uint32_t)sizeof(PlcPersistHeader) + len;
    const uint32_t padded = (total + (FLASHWORD_SIZE - 1u)) & ~(FLASHWORD_SIZE - 1u);

    if (padded > PLC_PERSIST_SIZE) {
        LOGT(TAG, "Persist blob save: too big padded=%lu > sector=%lu",
             (unsigned long)padded, (unsigned long)PLC_PERSIST_SIZE);
        return false;
    }

    if (flash_erase_sector(PLC_PERSIST_BASE) != HAL_OK) {
        LOGT(TAG, "Persist blob save: erase failed");
        return false;
    }

    if (flash_write_stream(PLC_PERSIST_BASE, &h, data, len) != HAL_OK) {
        LOGT(TAG, "Persist blob save: program failed");
        return false;
    }

    LOGT(TAG, "Persist blob saved: len=%lu padded=%lu", (unsigned long)len, (unsigned long)padded);
    return true;
}

bool plc_persist_clear(void) {
    return (flash_erase_sector(PLC_PERSIST_BASE) == HAL_OK);
}

// =====================
// High-level: SAVE/LOAD ACTIVE GRAPH
// =====================

// сохраняем только "статическую" часть узла, без runtime
typedef struct __attribute__((packed)) {
    uint16_t id;
    uint8_t type;
    uint8_t valueType;
    int16_t inA;
    int16_t inB;
    int32_t paramInt;
    float paramFloat;
    uint32_t paramMs;
    uint32_t flags;
} PlcNodeStatic;

typedef struct __attribute__((packed)) {
    uint32_t cycleMs;
    uint16_t nodeCount;
    uint16_t reserved;
    PlcNodeStatic nodes[PLC_MAX_NODES];
} PlcGraphStaticV1;

// Реальный максимум blob под граф (а не 128KB)
#define PLC_PERSIST_GRAPH_MAX_LEN \
    ((uint32_t)offsetof(PlcGraphStaticV1, nodes) + (uint32_t)(PLC_MAX_NODES) * (uint32_t)sizeof(PlcNodeStatic))

bool plc_persist_save(void) {
    if (!g_activeGraphValid) {
        LOGT(TAG, "save_active: no active graph");
        return false;
    }
    if (g_activeGraph.nodeCount == 0u || g_activeGraph.nodeCount > PLC_MAX_NODES) {
        LOGT(TAG, "save_active: bad nodeCount=%u", (unsigned)g_activeGraph.nodeCount);
        return false;
    }

    static PlcGraphStaticV1 st;
    memset(&st, 0, offsetof(PlcGraphStaticV1, nodes) + st.nodeCount * sizeof(PlcNodeStatic));
    st.cycleMs = g_activeGraph.cycleMs;
    st.nodeCount = g_activeGraph.nodeCount;

    for (uint16_t i = 0; i < st.nodeCount; i++) {
        PlcNode *n = &g_activeGraph.nodes[i];
        PlcNodeStatic *s = &st.nodes[i];

        s->id = n->id;
        s->type = n->type;
        s->valueType = n->valueType;
        s->inA = n->inA;
        s->inB = n->inB;
        s->paramInt = n->paramInt;
        s->paramFloat = n->paramFloat;
        s->paramMs = n->paramMs;
        s->flags = n->flags;
    }

    uint32_t blobLen = (uint32_t)(offsetof(PlcGraphStaticV1, nodes) + st.nodeCount * sizeof(PlcNodeStatic));
    return plc_persist_save_blob(&st, blobLen);
}

bool plc_persist_load(void) {
    // НЕ stack, иначе HardFault. Буфер ровно под максимально возможный граф.
    static uint8_t buf[PLC_PERSIST_GRAPH_MAX_LEN] __attribute__((aligned(32)));
    uint32_t len = 0;

    if (!plc_persist_load_blob(buf, sizeof(buf), &len)) {
        LOGT(TAG, "load_active: no blob");
        return false;
    }

    if (len < offsetof(PlcGraphStaticV1, nodes)) {
        LOGT(TAG, "load_active: blob too small len=%lu", (unsigned long)len);
        return false;
    }

    const PlcGraphStaticV1 *st = (const PlcGraphStaticV1 *)buf;

    if (st->nodeCount == 0u || st->nodeCount > PLC_MAX_NODES) {
        LOGT(TAG, "load_active: bad nodeCount=%u", (unsigned)st->nodeCount);
        return false;
    }

    uint32_t needLen = (uint32_t)(offsetof(PlcGraphStaticV1, nodes) + st->nodeCount * sizeof(PlcNodeStatic));
    if (len != needLen) {
        LOGT(TAG, "load_active: len mismatch got=%lu need=%lu",
             (unsigned long)len, (unsigned long)needLen);
        return false;
    }

    static PlcGraph g;     // <-- было на стеке
    memset(&g, 0, sizeof(g));
    g.cycleMs = st->cycleMs;
    g.nodeCount = st->nodeCount;

    for (uint16_t i = 0; i < g.nodeCount; i++) {
        const PlcNodeStatic *s = &st->nodes[i];
        PlcNode *n = &g.nodes[i];

        n->id = s->id;
        n->type = s->type;
        n->valueType = s->valueType;
        n->inA = s->inA;
        n->inB = s->inB;
        n->paramInt = s->paramInt;
        n->paramFloat = s->paramFloat;
        n->paramMs = s->paramMs;
        n->flags = s->flags;
        // runtime поля = 0 -> корректный старт
    }

    if (!plc_upload_graph(&g)) {
        LOGT(TAG, "load_active: plc_upload_graph failed");
        return false;
    }
    if (!plc_request_activate_graph()) {
        LOGT(TAG, "load_active: activate failed");
        return false;
    }

    LOGT(TAG, "load_active: OK (nodes=%u cycleMs=%lu)",
         (unsigned)g.nodeCount, (unsigned long)g.cycleMs);
    return true;
}
