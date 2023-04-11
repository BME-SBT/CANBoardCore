#ifndef UTIL_H
#define UTIL_H

#include "platform/status.h"

#define MAKE_NOCOPY(clazz)                                                     \
  private:                                                                     \
    clazz(const clazz &o) = delete;                                            \
    clazz &operator=(const clazz &o) = delete;

#define MAKE_NOMOVE(clazz)                                                     \
  private:                                                                     \
    clazz(const clazz &&o) = delete;                                           \
    clazz &operator=(const clazz &&o) = delete;

#define read_poll_timeout_blocking(op, value, condition, timeout_ms, sleep_ms, \
                                   args...)                                    \
    ({                                                                         \
        u64 start_time = millis();                                             \
        u64 deadline = start_time + timeout_ms;                                \
        while (millis() <= deadline) {                                         \
            (value) = op(args);                                                \
            if (condition) {                                                   \
                break;                                                         \
            }                                                                  \
            if (millis() > deadline) {                                         \
                break;                                                         \
            }                                                                  \
            delay(sleep_ms);                                                   \
        }                                                                      \
        (condition) ? PlatformStatus::STATUS_OK                                \
                    : PlatformStatus::STATUS_PLATFORM_POLL_TIMEOUT;            \
    })

#define TRY(expr)                                                              \
    do {                                                                       \
        PlatformStatus ret = (expr);                                           \
        if (is_err(ret)) {                                                     \
            return ret;                                                        \
        }                                                                      \
    } while (0)
#endif

#define schedule_call(fn, period) \
    static u64 fn##_last_called = 0; \
    if(millis() > fn##_last_called + period) { \
        fn();                     \
        fn##_last_called = millis();                              \
    }
