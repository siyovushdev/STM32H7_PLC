// plc_persist.c (STM32H7xx)
// Internal Flash persist implementation for STM32H743 (Cortex-M7, HAL)
// Writes are FLASHWORD (32 bytes), erase is sector-based, supports bank1/bank2.
//
// Replace your plc_persist.c with this file.
//
// Requirements:
// - stm32h7xx_hal.h included
// - logf.h provides LOGT(tag, fmt, ...) macro
//
// Note:
// - Choose safe flash area (usually last sector of Bank2 or dedicated sector).
// - Ensure linker script does NOT place code/data in this area.

#include "main.h"
#include "logf.h"
#include "plc_persist.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define TAG "PERSIST"

// ----------------------------
// CONFIG: choose persist region
// ----------------------------
// STM32H743: two banks, each bank typically 8 sectors x 128KB (0x20000).
// Bank1 base: 0x08000000
// Bank2 base: 0x08100000 (for 2MB total, bank size 1MB).
//
// !!! Set these to your desired area !!!
// Example: last sector of Bank2 (sector 7) start = 0x081E0000 (if 2MB device).
#ifndef PLC_PERSIST_BASE
#define PLC_PERSIST_BASE      (0x081E0000u)  // <-- CHANGE if needed
#endif

#ifndef PLC_PERSIST_SIZE
#define PLC_PERSIST_SIZE      (0x00020000u)  // 128KB = one sector (0x20000)
#endif

// Optional: simple header to validate stored blob
#define PLC_PERSIST_MAGIC     (0x504C4350u)  // 'PLCP'
#define PLC_PERSIST_VERSION   (1u)

// Max payload stored (within one sector in this example)
#ifndef PLC_PERSIST_MAX_DATA
#define PLC_PERSIST_MAX_DATA  (PLC_PERSIST_SIZE - 64u)
#endif

// ----------------------------
// H7 Flash constants
// ----------------------------
#define H7_SECTOR_SIZE        (0x20000u)      // 128KB per sector (typical for H743)
#define FLASHWORD_SIZE        (32u)           // 256-bit = 32 bytes

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t data_len;     // bytes
    uint32_t crc32;        // optional, can be 0 if not used
    uint32_t reserved[4];  // align to 32 bytes header-ish
} PlcPersistHeader;

// ----------------------------
// Small CRC32 (optional)
// If you already have CRC elsewhere, replace this.
// ----------------------------
static uint32_t crc32_simple(const void* data, uint32_t len) {
    const uint8_t* p = (const uint8_t*)data;
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

// ----------------------------
// Helpers: bank/sector
// ----------------------------
static inline uint32_t _addr_to_bank(uint32_t addr) {
    // Bank2 base macro exists in HAL for H7
    return (addr < FLASH_BANK2_BASE) ? FLASH_BANK_1 : FLASH_BANK_2;
}

static inline uint32_t _bank_base(uint32_t bank) {
    return (bank == FLASH_BANK_1) ? FLASH_BANK1_BASE : FLASH_BANK2_BASE;
}

static inline uint32_t _addr_to_sector(uint32_t addr) {
    uint32_t bank = _addr_to_bank(addr);
    uint32_t base = _bank_base(bank);
    return (addr - base) / H7_SECTOR_SIZE;
}

static void _log_flash_flags(void) {
    // Bank1 flags
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR_BANK1)) LOGT(TAG, "FLASH: WRPERR_BANK1");
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGSERR_BANK1)) LOGT(TAG, "FLASH: PGSERR_BANK1");
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_STRBERR_BANK1)) LOGT(TAG, "FLASH: STRBERR_BANK1");
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_INCERR_BANK1)) LOGT(TAG, "FLASH: INCERR_BANK1");
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPERR_BANK1))  LOGT(TAG, "FLASH: OPERR_BANK1");
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_RDPERR_BANK1)) LOGT(TAG, "FLASH: RDPERR_BANK1");
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP_BANK1))    LOGT(TAG, "FLASH: EOP_BANK1");

    // Bank2 flags
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR_BANK2)) LOGT(TAG, "FLASH: WRPERR_BANK2");
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGSERR_BANK2)) LOGT(TAG, "FLASH: PGSERR_BANK2");
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_STRBERR_BANK2)) LOGT(TAG, "FLASH: STRBERR_BANK2");
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_INCERR_BANK2)) LOGT(TAG, "FLASH: INCERR_BANK2");
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPERR_BANK2))  LOGT(TAG, "FLASH: OPERR_BANK2");
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_RDPERR_BANK2)) LOGT(TAG, "FLASH: RDPERR_BANK2");
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP_BANK2))    LOGT(TAG, "FLASH: EOP_BANK2");
}

static void _clear_flash_flags(uint32_t bank) {
    if (bank == FLASH_BANK_1) {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1 | FLASH_FLAG_EOP_BANK1);
    } else {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2 | FLASH_FLAG_EOP_BANK2);
    }
}

// ------------------------------------------------------
// Your old code had _log_flash_sr(FLASH->SR). On H7 we
// don't use SR directly. Keep same function name but
// accept dummy value and log via HAL instead.
// ------------------------------------------------------
static void _log_flash_sr(uint32_t unused_sr) {
    (void)unused_sr;
    uint32_t err = HAL_FLASH_GetError();
    LOGT(TAG, "HAL_FLASH_GetError=0x%08lX", (unsigned long)err);
    _log_flash_flags();
}

// ----------------------------
// Low-level erase sector (H7)
// ----------------------------
static HAL_StatusTypeDef flash_erase_sector(uint32_t addr) {
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sectorError = 0;

    const uint32_t bank = _addr_to_bank(addr);
    const uint32_t sector = _addr_to_sector(addr);

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks     = bank;
    erase.Sector    = sector;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; // typical 2.7..3.6V

    HAL_FLASH_Unlock();
    _clear_flash_flags(bank);

    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &sectorError);
    if (st != HAL_OK) {
        LOGT(TAG, "Erase fail: addr=0x%08lX bank=%lu sector=%lu sectorError=%lu",
             (unsigned long)addr, (unsigned long)bank, (unsigned long)sector, (unsigned long)sectorError);
        _log_flash_sr(0);
    }

    HAL_FLASH_Lock();
    return st;
}

// -----------------------------------------
// Write aligned by FLASHWORD (32 bytes)
// addr must be 32-byte aligned
// len must be multiple of 32
// -----------------------------------------
static HAL_StatusTypeDef flash_write_aligned(uint32_t addr, const void* data, uint32_t len) {
    if ((addr % FLASHWORD_SIZE) != 0u) {
        LOGT(TAG, "Write: addr not 32B aligned: 0x%08lX", (unsigned long)addr);
        return HAL_ERROR;
    }
    if ((len % FLASHWORD_SIZE) != 0u) {
        LOGT(TAG, "Write: len not multiple of 32: %lu", (unsigned long)len);
        return HAL_ERROR;
    }

    const uint32_t bank = _addr_to_bank(addr);

    HAL_FLASH_Unlock();
    _clear_flash_flags(bank);

    const uint8_t* p = (const uint8_t*)data;
    HAL_StatusTypeDef st = HAL_OK;

    for (uint32_t off = 0; off < len; off += FLASHWORD_SIZE) {
        // HAL expects a pointer-sized value cast to uint32_t
        // Buffer must be accessible and aligned enough. We'll use a local aligned buffer.
        uint32_t fw[8] __attribute__((aligned(4)));
        memcpy(fw, p + off, FLASHWORD_SIZE);

        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr + off, (uint32_t)fw);
        if (st != HAL_OK) {
            LOGT(TAG, "Program fail: addr=0x%08lX off=%lu", (unsigned long)(addr + off), (unsigned long)off);
            _log_flash_sr(0);
            break;
        }
    }

    HAL_FLASH_Lock();
    return st;
}

// -----------------------------------------
// Public API (simple blob store)
// -----------------------------------------
static inline const uint8_t* _flash_ptr(uint32_t addr) {
    return (const uint8_t*)(uintptr_t)addr;
}

bool plc_persist_load(void* out_buf, uint32_t out_max, uint32_t* out_len) {
    if (!out_buf || out_max == 0) return false;

    const PlcPersistHeader* h = (const PlcPersistHeader*)_flash_ptr(PLC_PERSIST_BASE);

    if (h->magic != PLC_PERSIST_MAGIC) {
        LOGT(TAG, "No persist: bad magic 0x%08lX", (unsigned long)h->magic);
        return false;
    }
    if (h->version != PLC_PERSIST_VERSION) {
        LOGT(TAG, "No persist: version %lu != %lu",
             (unsigned long)h->version, (unsigned long)PLC_PERSIST_VERSION);
        return false;
    }
    if (h->data_len == 0 || h->data_len > PLC_PERSIST_MAX_DATA || h->data_len > out_max) {
        LOGT(TAG, "No persist: bad len=%lu", (unsigned long)h->data_len);
        return false;
    }

    const uint8_t* data = _flash_ptr(PLC_PERSIST_BASE + sizeof(PlcPersistHeader));
    uint32_t crc = crc32_simple(data, h->data_len);
    if (h->crc32 != 0 && crc != h->crc32) {
        LOGT(TAG, "No persist: CRC mismatch calc=0x%08lX stored=0x%08lX",
             (unsigned long)crc, (unsigned long)h->crc32);
        return false;
    }

    memcpy(out_buf, data, h->data_len);
    if (out_len) *out_len = h->data_len;

    LOGT(TAG, "Persist loaded: len=%lu", (unsigned long)h->data_len);
    return true;
}

bool plc_persist_save(const void* data, uint32_t len) {
    if (!data || len == 0 || len > PLC_PERSIST_MAX_DATA) {
        LOGT(TAG, "Persist save: bad args len=%lu", (unsigned long)len);
        return false;
    }

    // Build header
    PlcPersistHeader h;
    memset(&h, 0, sizeof(h));
    h.magic   = PLC_PERSIST_MAGIC;
    h.version = PLC_PERSIST_VERSION;
    h.data_len = len;
    h.crc32   = crc32_simple(data, len);

    // We must write in 32-byte units. We'll create a staging buffer in RAM:
    // [header][data][padding] aligned to 32.
    uint32_t total = (uint32_t)sizeof(PlcPersistHeader) + len;
    uint32_t padded = (total + (FLASHWORD_SIZE - 1u)) & ~(FLASHWORD_SIZE - 1u);

    // Safety: one-sector store
    if (padded > PLC_PERSIST_SIZE) {
        LOGT(TAG, "Persist save: too big padded=%lu > sector=%lu", (unsigned long)padded, (unsigned long)PLC_PERSIST_SIZE);
        return false;
    }

    // Allocate on stack if small, else you can move to static buffer.
    // For PLC graphs it might be big; if you exceed stack, switch to static.
    // Here we use a static buffer capped to one sector.
    static uint8_t s_buf[PLC_PERSIST_SIZE] __attribute__((aligned(32)));

    memset(s_buf, 0xFF, padded);
    memcpy(s_buf, &h, sizeof(h));
    memcpy(s_buf + sizeof(h), data, len);

    // Erase sector
    if (flash_erase_sector(PLC_PERSIST_BASE) != HAL_OK) {
        LOGT(TAG, "Persist save: erase failed");
        return false;
    }

    // Program
    if (flash_write_aligned(PLC_PERSIST_BASE, s_buf, padded) != HAL_OK) {
        LOGT(TAG, "Persist save: program failed");
        return false;
    }

    LOGT(TAG, "Persist saved: len=%lu padded=%lu", (unsigned long)len, (unsigned long)padded);
    return true;
}

// Optional: clear persist (erase sector)
bool plc_persist_clear(void) {
    return (flash_erase_sector(PLC_PERSIST_BASE) == HAL_OK);
}
