#ifndef UTIL_H
#define UTIL_H

#define MAKE_NOCOPY(clazz)          \
private:                            \
    clazz(const clazz &o) = delete; \
    clazz &operator=(const clazz &o) = delete;

#define MAKE_NOMOVE(clazz)           \
private:                             \
    clazz(const clazz &&o) = delete; \
    clazz &operator=(const clazz &&o) = delete;

#define read_poll_timeout_blocking(op, value, condition, timeout_ms, sleep_ms, args...) \
    ({                                                                                  \
        u64 start_time = millis();                                                      \
        u64 deadline = start_time + timeout_ms;                                         \
        while (millis() <= deadline)                                                    \
        {                                                                               \
            (value) = op(args);                                                         \
            if (condition)                                                              \
            {                                                                           \
                break;                                                                  \
            }                                                                           \
            if (millis() > deadline)                                                    \
            {                                                                           \
                break;                                                                  \
            }                                                                           \
            delay(sleep_ms);                                                            \
        }                                                                               \
        (condition) ? 0 : STATUS_POLL_TIMEOUT;                                          \
    })

#endif