#include "driver/can/can.h"
#include "driver/can/canstat.h"
#include "driver/can/spi_mcp2510.h"
#include "platform/platform.h"

CAN_Stat g_can_stat{};

bool CAN::init()
{
    m_driver = mcp251x_platform_init(PLATFORM_CAN_OSCILLATOR, PLATFORM_CAN_BAUD);
    if (!m_driver)
    {
        failed = true;
    }
    return failed;
}

bool CAN::available()
{

    has_frame = m_queue.pull(&this->current_frame);
    if(has_frame){
        g_can_stat.rx_processed_frame_count++;
    }
    return has_frame;
}

int CAN::send(CAN_Frame frame)
{

    int err = mcp251x_start_tx(m_driver, frame);
    if (err == CAN_BUSY)
    {
        // overwrite the last one, newer frames are always prioritized
        if(m_queue.insert(frame)) {
            g_can_stat.tx_queued_frame_count++;
            return CAN_QUEUED;
        }else {
            g_can_stat.tx_dropped_frame_count++;
            return CAN_DROPPED;
        }
    }
    else if(err)
    {
        return err;
    }
    return 0;
}

/**
 * Try sending tx queue
 */
int CAN::transmit_tick()
{
    CAN_Frame *frame;
    while ((frame = m_queue.peek()) != nullptr)
    {
        int err = mcp251x_start_tx(m_driver, *frame);
        if (!err)
        {
            m_queue.pull(nullptr);
        }
        else if (err == CAN_BUSY)
        {
            break;
        }
        else
        {
            return err;
        }
    }

    return 0;
}