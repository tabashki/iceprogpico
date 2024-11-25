#include <pico/stdio.h>
#include <pico/time.h>
#include <string.h>
#include <stdio.h>

#include "log.h"

// Logging support for debugging
//===============================

#define LOG_MAX_MESSAGE_LEN 58  // Excluding NUL terminator
#define LOG_QUEUE_DEPTH 128

#define ADVANCE_QUEUE_PTR(p) do { (p) = ((p) + 1) % LOG_QUEUE_DEPTH; } while(0)

typedef struct {
    uint32_t timestamp_us;
    uint8_t severity;
    char message[LOG_MAX_MESSAGE_LEN + 1];  // +1 for the NUL terminator
} log_entry;

static_assert(sizeof(log_entry) == 64);

#if WITH_LOGGING

// Circular buffer of log message entries
log_entry log_queue[LOG_QUEUE_DEPTH];
uint8_t log_queue_read_ptr = 0;
uint8_t log_queue_write_ptr = 0;
uint32_t log_queue_dropped = 0;

// Internal Functions
//--------------------

static void log_push_entry(uint8_t severity, const char* fmt, va_list fmt_args) {
    log_entry* new_entry = &log_queue[log_queue_write_ptr];
    new_entry->timestamp_us = time_us_32();
    new_entry->severity = severity;

    int n = vsnprintf(new_entry->message, sizeof(new_entry->message),
                      fmt, fmt_args);
    n = MAX(n, 0);
    n = MIN(n, LOG_MAX_MESSAGE_LEN);
    new_entry->message[n] = 0;  // Ensure we have a NUL terminator for safety

    ADVANCE_QUEUE_PTR(log_queue_write_ptr);
    if (log_queue_read_ptr == log_queue_write_ptr) {
        // Queue is full, drop the oldest entry
        log_queue_dropped++;
        ADVANCE_QUEUE_PTR(log_queue_read_ptr);
    }
}

static const char* log_severity_to_str(log_severity s) {
    switch (s) {
    case LOG_SEVERITY_INFO: return "INFO";
    case LOG_SEVERITY_WARN: return "WARN";
    case LOG_SEVERITY_ERROR: return "ERROR";
    default: return "???";
    }
}

// Public Functions
//------------------

void log_clear() {
    log_queue_read_ptr = 0;
    log_queue_write_ptr = 0;
    log_queue_dropped = 0;
}

void log_sink_to_stdio() {
    if (log_queue_dropped > 0) {
        stdio_printf("(... %lu piror messages dropped)\n", log_queue_dropped);
    }
    while(log_queue_read_ptr != log_queue_write_ptr) {
        const log_entry* entry = &log_queue[log_queue_read_ptr];

        uint32_t secs = entry->timestamp_us / 1000000u;
        uint32_t micros = entry->timestamp_us % 1000000u;
        const char* sev_str = log_severity_to_str(entry->severity);
        const char* msg = entry->message;

        stdio_printf("[%lu.%06lu] (%s) %s\n", secs, micros, sev_str, msg);
        ADVANCE_QUEUE_PTR(log_queue_read_ptr);
    }
    log_queue_dropped = 0;
}

void log_with_severity(log_severity s, const char *fmt, ...) {
    va_list va;
    va_start(va, fmt);
    log_push_entry(s, fmt, va);
    va_end(va);
}

#endif // WITH_LOGGING
