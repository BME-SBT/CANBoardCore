#ifndef LOG_H
#define LOG_H

#define LOG_ENABLED 0
#define LOG_PRINTF_BUF_SIZE 128
#define LOG_IRQ_PRINTF_SIZE 1024

#if LOG_ENABLED
#include <string.h>
#define __FNAME__                                                              \
    (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__)

extern char printf_buffer[LOG_PRINTF_BUF_SIZE];
extern char irq_printf_buffer1[LOG_PRINTF_BUF_SIZE];
extern char irq_printf_buffer2[LOG_PRINTF_BUF_SIZE + 7];
extern char irq_logbuf[LOG_IRQ_PRINTF_SIZE];

void afterirq_log();

#define log(msg)                                                               \
    do {                                                                       \
        SerialUSB.print("[");                                                  \
        SerialUSB.print(millis() / 1000.0);                                    \
        SerialUSB.print("] ");                                                 \
        SerialUSB.print(__FNAME__);                                            \
        SerialUSB.print(":");                                                  \
        SerialUSB.print(__LINE__);                                             \
        SerialUSB.print(": ");                                                 \
        SerialUSB.println(msg);                                                \
    } while (0)
#define logf(format, args...)                                                  \
    do {                                                                       \
        snprintf(printf_buffer, LOG_PRINTF_BUF_SIZE, format, args);            \
        log(printf_buffer);                                                    \
    } while (0)

#define irqlogf(format, args...)                                               \
    do {                                                                       \
        snprintf(irq_printf_buffer1, LOG_PRINTF_BUF_SIZE, format, args);       \
        snprintf(irq_printf_buffer2, LOG_PRINTF_BUF_SIZE + 7, "IRQ: %s\r\n",   \
                 irq_printf_buffer1);                                          \
        unsigned int csize = strlen(irq_logbuf);                               \
        strncat(irq_logbuf, irq_printf_buffer2, LOG_IRQ_PRINTF_SIZE - csize);  \
    } while (0)

#define irqlog(message)                                                        \
    do {                                                                       \
        snprintf(irq_printf_buffer2, LOG_PRINTF_BUF_SIZE + 7, "IRQ: %s\r\n",   \
                 message);                                                     \
        unsigned int csize = strlen(irq_logbuf);                               \
        strncat(irq_logbuf, irq_printf_buffer2, LOG_IRQ_PRINTF_SIZE - csize);  \
    } while (0)

#define log_if(flag, msg)                                                      \
    do {                                                                       \
        if (flag)                                                              \
            log(msg);                                                          \
    } while (0)

#define logf_if(flag, format, args...)                                         \
    do {                                                                       \
        if (flag)                                                              \
            logf(format, args);                                                \
    } while (0)
#define log_init()  memset(irq_logbuf, 0,LOG_IRQ_PRINTF_SIZE)
#else
#define log(msg)
#define logf(fmt, args...)
#define irqlog(msg)
#define irqlogf(fmt, args...)
#define log_if(flag, msg)
#define logf_if(flag, fmt, args...)
#define log_init()
inline void afterirq_log() {}
#endif

#endif