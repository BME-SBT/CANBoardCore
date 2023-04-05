#ifndef CAN_H
#define CAN_H

#include "spi_mcp2510.h"

#define CAN_QUEUED 0xff
#define CAN_QUEUE_FULL 0xfc

class CAN
{
public:
    CAN() : failed(true), has_frame(false)
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
    Priv *m_driver;
    bool failed;
    CAN_Frame current_frame;
    bool has_frame;
    queue_t tx_queue;
};

#endif