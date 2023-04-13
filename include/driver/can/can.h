#ifndef CAN_H
#define CAN_H

#include "canstat.h"
#include "driver/can/canpriorityqueue.h"
#include "spi_mcp2510.h"
#include "can_frame.h"

#define CAN_QUEUED 0xff
#define CAN_DROPPED 0xfe

class CAN
{
public:
    CAN() : m_driver(), failed(true), has_frame(false)
    {
    }

    bool init();

    bool available();

    CAN_Frame get_frame()
    {
        return current_frame;
    }

    int send(CAN_Frame frame);

    /**
     * Try sending tx queue
     */
    int transmit_tick();

private:
    Priv m_driver;
    bool failed;
    CAN_Frame current_frame{};
    bool has_frame;
    // In theory, IRQ can send CAN frames
};

#endif