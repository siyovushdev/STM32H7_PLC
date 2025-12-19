#include "logf.h"

volatile uint8_t     g_log_enabled = 1;
volatile log_level_t g_log_level   = LOG_LEVEL_DEBUG;
const char*          g_log_tag     = "APP";

void logf_set_enabled(bool en)      { g_log_enabled = en ? 1 : 0; }
void logf_set_level(log_level_t lv) { g_log_level = lv; }
void logf_set_tag(const char* tag)  { g_log_tag = tag ? tag : "APP"; }

#if LOG_ENABLE
static const char* _lvl_name(log_level_t l){
    switch(l){
        case LOG_LEVEL_ERROR: return "E";
        case LOG_LEVEL_WARN:  return "W";
        case LOG_LEVEL_INFO:  return "I";
        default:              return "D";
    }
}

void logf_emit(log_level_t lvl, const char* func, int line, const char* fmt, ...){
    if (!g_log_enabled || lvl > g_log_level) return;
    // Формат: TAG L FUNC:LINE message
    printf("%s %s %s:%d ", g_log_tag, _lvl_name(lvl), func, line);
    va_list ap; va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\r\n");
}

void logf_emitByTag(const char* tag, const char* func, int line, const char* fmt, ...){
    if (!g_log_enabled) return;
    // Формат: TAG L FUNC:LINE message
    printf("%s %s %s:%d ", tag, "D", func, line);
    va_list ap; va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\r\n");
}
#else
void logf_emit(log_level_t lvl, const char* func, int line, const char* fmt, ...){ (void)lvl;(void)func;(void)line;(void)fmt; }
#endif
