#ifndef STATUS_H
#define STATUS_H

#include "gen_status.h"

inline bool is_err(PlatformStatus status) {
    return status != PlatformStatus::STATUS_OK;
}

inline int statuscode(PlatformStatus status) {
    return static_cast<int>(status);
}

#endif