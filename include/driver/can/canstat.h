//
// Created by Barrow099 on 2023. 04. 08..
//

#ifndef CANBOARDCOMMON_CANSTAT_H
#define CANBOARDCOMMON_CANSTAT_H

#include "lib/inttypes.h"

struct CAN_Stat {
    u32 tx_dropped_frame_count;
    u32 tx_queued_frame_count;
    u32 tx_sent_frame_count;
    u32 rx_dropped_frame_count;
    u32 rx_received_frame_count;
    u32 rx_processed_frame_count;
    u32 isr_lost_race_count;
};

extern CAN_Stat g_can_stat;

#endif // CANBOARDCOMMON_CANSTAT_H
