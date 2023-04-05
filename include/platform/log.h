#ifndef LOG_H
#define LOG_H

#define LOG_ENABLED 1
#define LOG_PRINTF_BUF_SIZE 128

#ifdef LOG_ENABLED
#include <string.h>
#define __FNAME__ (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__)

extern char printf_buffer[LOG_PRINTF_BUF_SIZE];
#define log(msg)                            \
    do                                      \
    {                                       \
        SerialUSB.print("[");               \
        SerialUSB.print(millis() / 1000.0); \
        SerialUSB.print("] ");              \
        SerialUSB.print(__FNAME__);         \
        SerialUSB.print(":");               \
        SerialUSB.print(__LINE__);          \
        SerialUSB.print(": ");              \
        SerialUSB.println(msg);             \
    } while (0)
#define logf(format, args...)                                       \
    do                                                              \
    {                                                               \
        snprintf(printf_buffer, LOG_PRINTF_BUF_SIZE, format, args); \
        log(printf_buffer);                                         \
    } while (0)

#define log_if(flag, msg) \
    do                    \
    {                     \
        if (flag)         \
            log(msg);     \
    } while (0)

#define logf_if(flag, format, args...) \
    do                                 \
    {                                  \
        if (flag)                      \
            logf(format, args);        \
    } while (0)

#else
#define log(msg)
#define logf(fmt, args...)
#endif

#endif