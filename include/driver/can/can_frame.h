//
// Created by Barrow099 on 2023. 04. 13..
//

#ifndef CANBOARDCOMMON_CAN_FRAME_H
#define CANBOARDCOMMON_CAN_FRAME_H

#include "lib/inttypes.h"
#include <cstring>

struct CAN_Frame {
    union {
        u16 standard_id;
        u32 extended_id;
    };
    bool rtr;
    bool extended;
    u8 dlc;
    u8 data[8];

    CAN_Frame() = default;

    CAN_Frame(u16 sid, u8 *data, u8 dlc) {
        this->standard_id = sid;
        memcpy(this->data, data, dlc);
        this->dlc = dlc;
    }

    u32 priority() const { return extended ? extended_id : standard_id; }
};

#endif // CANBOARDCOMMON_CAN_FRAME_H
