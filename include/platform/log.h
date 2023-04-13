#ifndef LOG_H
#define LOG_H

#define LOG_ENABLED 0
#define LOG_PRINTF_BUF_SIZE 128
#define LOG_IRQ_PRINTF_SIZE 1024

#if LOG_ENABLED

#include <Arduino.h>
#include <cstdio>
#include <cstring>
#define __FILENAME__                                                           \
    (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__)

extern char printf_buffer[LOG_PRINTF_BUF_SIZE];
extern char irq_printf_buffer1[LOG_PRINTF_BUF_SIZE];
extern char irq_printf_buffer2[LOG_PRINTF_BUF_SIZE + 7];
extern char irq_logbuf[LOG_IRQ_PRINTF_SIZE];

void afterirq_log();

#define LOG(msg)                                                               \
    do {                                                                       \
        SerialUSB.print("[");                                                  \
        SerialUSB.print(millis() / 1000.0);                                    \
        SerialUSB.print("] ");                                                 \
        SerialUSB.print(__FILENAME__);                                         \
        SerialUSB.print(":");                                                  \
        SerialUSB.print(__LINE__);                                             \
        SerialUSB.print(": ");                                                 \
        SerialUSB.println(msg);                                                \
    } while (0)

#define LOGF(format, args...)                                                  \
    do {                                                                       \
        snprintf(printf_buffer, LOG_PRINTF_BUF_SIZE, format, args);            \
        LOG(printf_buffer);                                                    \
    } while (0)

#define IRQ_LOGF(format, args...)                                              \
    do {                                                                       \
        snprintf(irq_printf_buffer1, LOG_PRINTF_BUF_SIZE, format, args);       \
        snprintf(irq_printf_buffer2, LOG_PRINTF_BUF_SIZE + 7, "IRQ: %s\r\n",   \
                 irq_printf_buffer1);                                          \
        unsigned int csize = strlen(irq_logbuf);                               \
        strncat(irq_logbuf, irq_printf_buffer2, LOG_IRQ_PRINTF_SIZE - csize);  \
    } while (0)

#define IRQ_LOG(message)                                                       \
    do {                                                                       \
        snprintf(irq_printf_buffer2, LOG_PRINTF_BUF_SIZE + 7, "IRQ: %s\r\n",   \
                 message);                                                     \
        unsigned int csize = strlen(irq_logbuf);                               \
        strncat(irq_logbuf, irq_printf_buffer2, LOG_IRQ_PRINTF_SIZE - csize);  \
    } while (0)

#define LOG_IF(flag, msg)                                                      \
    do {                                                                       \
        if (flag)                                                              \
            LOG(msg);                                                          \
    } while (0)

#define LOGF_IF(flag, format, args...)                                         \
    do {                                                                       \
        if (flag)                                                              \
            LOGF(format, args);                                                \
    } while (0)

#define LOG_INIT() memset(irq_logbuf, 0, LOG_IRQ_PRINTF_SIZE)
#else
#define LOG(msg)
#define LOGF(fmt, args...)
#define IRQ_LOG(msg)
#define IRQ_LOGF(fmt, args...)
#define LOG_IF(flag, msg)
#define LOGF_IF(flag, fmt, args...)
#define LOG_INIT()
inline void afterirq_log() {}
#endif

#endif