#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// =====================
// Flash persist region
// =====================
// STM32H743 (2MB Flash): Bank1 0x0800_0000..0x080F_FFFF, Bank2 0x0810_0000..0x081F_FFFF
// Сектор обычно 128KB (0x20000). Последний сектор Bank2 стартует с 0x081E0000.
//
// ВАЖНО: зарезервируй этот диапазон в линкере (см. ниже).

#ifndef PLC_PERSIST_BASE
#define PLC_PERSIST_BASE      (0x081E0000u)   // last 128KB sector of Bank2
#endif

#ifndef PLC_PERSIST_SIZE
#define PLC_PERSIST_SIZE      (0x00020000u)   // 128KB
#endif

// Макс размер сохраняемых данных (внутри сектора)
#ifndef PLC_PERSIST_MAX_DATA
#define PLC_PERSIST_MAX_DATA  (PLC_PERSIST_SIZE - 64u)
#endif

// =====================
// High-level API (как ожидает cbor_msg.c)
// =====================

// save current g_activeGraph into flash
bool plc_persist_save(void);

// load graph from flash -> upload into staging -> activate
bool plc_persist_load(void);

// erase persist sector
bool plc_persist_clear(void);

// =====================
// Low-level blob API (если понадобится позже)
// =====================
bool plc_persist_save_blob(const void* data, uint32_t len);
bool plc_persist_load_blob(void* out_buf, uint32_t out_max, uint32_t* out_len);

#ifdef __cplusplus
}
#endif
