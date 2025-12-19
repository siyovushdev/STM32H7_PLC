#ifndef LOGF_H
#define LOGF_H

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- Compile-time выключатель (0 -> весь код логов вырезается оптимизацией) ---
#ifndef LOG_ENABLE
#define LOG_ENABLE 1
#endif

// --- Уровни логов ---
typedef enum {
    LOG_LEVEL_ERROR = 0,
    LOG_LEVEL_WARN  = 1,
    LOG_LEVEL_INFO  = 2,
    LOG_LEVEL_DEBUG = 3,
} log_level_t;

// --- Глобальные настройки (runtime) ---
extern volatile uint8_t   g_log_enabled;  // 0/1
extern volatile log_level_t g_log_level;  // текущий уровень
extern const char*        g_log_tag;      // строка тега

void logf_set_enabled(bool en);
void logf_set_level(log_level_t lvl);
void logf_set_tag(const char* tag);

// --- Базовый эмиттер (не используйте напрямую) ---
void logf_emit(log_level_t lvl, const char* func, int line, const char* fmt, ...);
void logf_emitByTag(const char* tag, const char* func, int line, const char* fmt, ...);

// --- Макросы уровней ---
#if LOG_ENABLE
#define LOGE(fmt, ...) logf_emit(LOG_LEVEL_ERROR, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) logf_emit(LOG_LEVEL_WARN,  __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) logf_emit(LOG_LEVEL_INFO,  __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) logf_emit(LOG_LEVEL_DEBUG, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGT(tag, fmt, ...) logf_emitByTag(tag, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)

#else
#define LOGE(...) do{}while(0)
  #define LOGW(...) do{}while(0)
  #define LOGI(...) do{}while(0)
  #define LOGD(...) do{}while(0)
  #define LOGT(...) do{}while(0)
#endif

#ifdef __cplusplus
}
#endif
#endif // LOGF_H
