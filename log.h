
#ifndef LOG_H
#define LOG_H

#include <pico/stdlib.h>

// Logging support for debugging
//===============================

#define WITH_LOGGING 1

typedef enum {
    LOG_SEVERITY_INFO,
    LOG_SEVERITY_WARN,
    LOG_SEVERITY_ERROR,
} log_severity;

#if WITH_LOGGING

void log_clear();
void log_sink_to_stdio();
void log_with_severity(log_severity s, const char* fmt, ...);

#define LOG_INFO(f, ...) \
    log_with_severity(LOG_SEVERITY_INFO, f, ## __VA_ARGS__)

#define LOG_WARN(f, ...) \
    log_with_severity(LOG_SEVERITY_WARN, f, ## __VA_ARGS__)

#define LOG_ERROR(f, ...) \
    log_with_severity(LOG_SEVERITY_ERROR, f, ## __VA_ARGS__)

// Conditional Logging Variants
//------------------------------
// NOTE: Ensure conditions do not have side-effects, they may be compiled out!

#define LOG_COND_INFO(cond, f, ...) do { if (!!(cond)) { \
        log_with_severity(LOG_SEVERITY_INFO, f, ## __VA_ARGS__); \
    } } while(0)

#define LOG_COND_WARN(cond, f, ...) do { if (!!(cond)) { \
        log_with_severity(LOG_SEVERITY_WARN, f, ## __VA_ARGS__); \
    } } while(0)

#define LOG_COND_ERROR(cond, f, ...) do { if (!!(cond)) { \
        log_with_severity(LOG_SEVERITY_ERROR, f, ## __VA_ARGS__); \
    } } while(0)

#else

inline void log_clear() {}
inline void log_sink_to_stdio() {}
inline void log_with_severity(log_severity s, const char* fmt, ...) {}

#define LOG_INFO(f, ...)  ((void)0)
#define LOG_WARN(f, ...)  ((void)0)
#define LOG_ERROR(f, ...) ((void)0)

#define LOG_COND_INFO(cond, f, ...)  ((void)0)
#define LOG_COND_WARN(cond, f, ...)  ((void)0)
#define LOG_COND_ERROR(cond, f, ...) ((void)0)

#endif // WITH_LOGGING

#endif // LOG_H
