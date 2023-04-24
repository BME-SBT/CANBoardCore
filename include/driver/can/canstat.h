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
    u32 rx_ov_dropped_frame_count;
    u32 rx_received_frame_count;
    u32 rx_processed_frame_count;
    u32 isr_lost_race_count;
    u32 irq_handled;
    u32 txb0_set;
    u32 txb1_set;
    u32 txb2_set;
    u32 txb0_cleared;
    u32 txb1_cleared;
    u32 txb2_cleared;
    u64 tx_byte_count;
    u64 rx_byte_count;
    u64 tx_req_byte_count;
};

extern CAN_Stat g_can_stat;

#endif // CANBOARDCOMMON_CANSTAT_H
