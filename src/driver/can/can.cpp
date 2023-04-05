#include "driver/can/can.h"
#include "driver/can/spi_mcp2510.h"
#include "platform/platform.h"

bool CAN::init()
{
    m_driver = mcp251x_platform_init(PLATFORM_CAN_OSCILLATOR, PLATFORM_CAN_BAUD);
    queue_init(&this->tx_queue, sizeof(CAN_Frame), 16);
    if (!m_driver)
    {
        failed = true;
    }
    return failed;
}

bool CAN::available()
{
    __disable_irq();
    has_frame = queue_try_remove(&m_driver->can_rx_queue, &this->current_frame);
    __enable_irq();
    return has_frame;
}

int CAN::send(CAN_Frame frame)
{
    int err = mcp251x_start_tx(m_driver, frame);
    if (err == CAN_BUSY)
    {
        if (queue_try_add(&this->tx_queue, &frame))
            return CAN_QUEUED;
        return CAN_QUEUE_FULL;
    }
    else
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
    CAN_Frame frame;
    while (queue_try_peek(&this->tx_queue, &frame))
    {
        int err = mcp251x_start_tx(m_driver, frame);
        if (!err)
        {
            queue_remove_blocking(&this->tx_queue, &frame);
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