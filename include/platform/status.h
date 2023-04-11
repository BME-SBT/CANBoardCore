#ifndef STATUS_H
#define STATUS_H

#ifdef DEBUG
#define STATUS_DEBUG
#endif
#include "gen_status.h"

inline bool is_err(PlatformStatus status) {
    return status != PlatformStatus::STATUS_OK;
}

inline int statuscode(PlatformStatus status) {
    return static_cast<int>(status);
}

inline const char* statusname(PlatformStatus status) {
    return status_lookup_code(statuscode(status));
}

#endif