#pragma once
#include <stdbool.h>


// ==== Конфигурация сектора (переопредели в проекте при необходимости) ====
// Для STM32F411(xE): Sector7 = 0x08060000, размер 128 KB.
// Для меньших чипов укажи существующий крупный сектор, не занятый кодом.
#ifndef PLC_FLASH_SECTOR_ID
#define PLC_FLASH_SECTOR_ID   FLASH_SECTOR_7
#endif

#ifndef PLC_FLASH_SECTOR_ADDR
#define PLC_FLASH_SECTOR_ADDR 0x08060000UL
#endif

#ifndef PLC_FLASH_SECTOR_SIZE
#define PLC_FLASH_SECTOR_SIZE (128UL*1024UL)
#endif

// Публичный API
//bool plc_persist_save_active(void);
//bool plc_persist_load_active(void);

// Можно дернуть из команд: {"cmd":"save_active"} / {"cmd":"load_active"}
