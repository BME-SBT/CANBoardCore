/*
 * This driver is based on the mcp251x.c driver from Linux and uses raw pico-sdk SPI access.
 *
 * This is both an SPI and a CAN driver
 */
#include "platform/platform.h"
#include "driver/can/can_timing.h"
#include <hardware/spi.h>
#include <pico/mutex.h>
#include <pico/util/queue.h>
#include <hardware/gpio.h>
#include "driver/can/spi_mcp2510.h"

static SPISettings spi_settings(1000000, BitOrder::MSBFIRST, SPIMode::SPI_MODE0);

static void mcp251x_can_isr(void *device);

static int mcp251x_spi_write(Priv *priv, size_t len)
{
    gpio_put(PLATFORM_PIN_CAN_CS, 0);
    PLATFORM_CAN_SPI.beginTransaction(spi_settings);
    PLATFORM_CAN_SPI.transfer(priv->spi_tx_buf, len);
    PLATFORM_CAN_SPI.endTransaction();
    gpio_put(PLATFORM_PIN_CAN_CS, 1);
    return 0;
}

static int mcp251x_spi_transfer(Priv *priv, size_t len)
{
    gpio_put(PLATFORM_PIN_CAN_CS, 0);
    PLATFORM_CAN_SPI.beginTransaction(spi_settings);
    for (size_t index = 0; index < len; index++)
    {
        priv->spi_rx_buf[index] = PLATFORM_CAN_SPI.transfer(priv->spi_tx_buf[index]);
    }
    gpio_put(PLATFORM_PIN_CAN_CS, 1);
    return 0;
}

static int mcp251x_read_reg(Priv *priv, u8 reg)
{
    priv->spi_tx_buf[0] = INSTRUCTION_READ;
    priv->spi_tx_buf[1] = reg;

    mcp251x_spi_transfer(priv, 3);
    return priv->spi_rx_buf[2];
}

static void mcp251x_write_reg(Priv *priv, u8 reg, u8 value)
{
    priv->spi_tx_buf[0] = INSTRUCTION_WRITE;
    priv->spi_tx_buf[1] = reg;
    priv->spi_tx_buf[2] = value;

    mcp251x_spi_write(priv, 3);
}

static void mcp251x_write_bits(Priv *priv, u8 reg, u8 mask, u8 value)
{
    priv->spi_tx_buf[0] = INSTRUCTION_BIT_MODIFY;
    priv->spi_tx_buf[1] = reg;
    priv->spi_tx_buf[2] = mask;
    priv->spi_tx_buf[3] = value;

    mcp251x_spi_write(priv, 4);
}

static u8 mcp251x_read_stat(Priv *priv)
{
    return mcp251x_read_reg(priv, CANSTAT) & CANCTRL_REQOP_MASK;
}

static int mcp251x_reset(Priv *priv)
{
    delay(5); // Wait for oscillator startup

    // send reset
    priv->spi_tx_buf[0] = INSTRUCTION_RESET;
    int ret = mcp251x_spi_write(priv, 1);
    if (ret)
    {
        return ret;
    }
    delay(5); // Wait for oscillator reset

    // wait for reset
    platform_set_status(STATUS_CAN_RESETWAIT); // signal waiting for can reset, we cannot proceed without CAN
    u8 value;
    ret = 0;
    ret = read_poll_timeout_blocking(mcp251x_read_stat, value, value == CANCTRL_REQOP_CONF, 1000, 10, priv);
    if (ret)
    {
        return ret;
    }
    return 0;
}

#define can_cc_dlc2len(dlc) (min((dlc), 8))

static void mcp251x_hw_rx_frame(Priv *priv, u8 *buf, int rxb)
{
    int i, len;

    for (i = 1; i < RXBDAT_OFF; i++)
        buf[i] = mcp251x_read_reg(priv, RXBCTRL(rxb) + i);

    len = can_cc_dlc2len(buf[RXBDLC_OFF] & RXBDLC_LEN_MASK);
    for (; i < (RXBDAT_OFF + len); i++)
        buf[i] = mcp251x_read_reg(priv, RXBCTRL(rxb) + i);
}

static void mcp251x_hw_rx(Priv *priv, int rxb)
{
    CAN_Frame skb;
    u8 buf[SPI_TRANSFER_BUF_LEN];

    mcp251x_hw_rx_frame(priv, buf, rxb);

    if (buf[RXBSIDL_OFF] & RXBSIDL_IDE)
    {
        // we got an extended frame
        skb.extended_id = 0;
        skb.extended_id |= (buf[RXBSIDL_OFF] & RXBSIDL_EID) << 16;
        skb.extended_id |= (buf[RXBEID8_OFF]) << 8;
        skb.extended_id |= (buf[RXBEID0_OFF]);
        skb.extended = true;
        if (buf[RXBDLC_OFF] & RXBDLC_RTR)
        {
            skb.rtr = true;
        }
    }
    else
    {
        // standard frame
        skb.extended = false;
        skb.standard_id = 0;
        skb.standard_id |= (buf[RXBSIDH_OFF] << RXBSIDH_SHIFT) | (buf[RXBSIDL_OFF] >> RXBSIDL_SHIFT);
        if (buf[RXBSIDL_OFF] & RXBSIDL_SRR)
        {
            skb.rtr = true;
        }
    }
    skb.dlc = can_cc_dlc2len(buf[RXBDLC_OFF] & RXBDLC_LEN_MASK);
    if (!(skb.rtr))
    {
        memcpy(skb.data, buf + RXBDAT_OFF, skb.dlc);
    }

    // TODO: Add packet to queue
    queue_try_add(&priv->can_rx_queue, &skb);
}

static void mcp251x_hw_tx_frame(Priv *priv, u8 *buf,
                                int len, int txb)
{
    int i;

    for (i = 1; i < TXBDAT_OFF + len; i++)
        mcp251x_write_reg(priv, TXBCTRL(txb) + i,
                          buf[i]);
}

static void mcp251x_hw_tx(Priv *priv, const CAN_Frame &frame, int txb)
{
    u32 sid, eid, exide, rtr;
    u8 buf[SPI_TRANSFER_BUF_LEN];

    exide = frame.extended;
    if (exide)
        sid = (frame.extended_id) >> 18;
    else
        sid = frame.standard_id;
    eid = frame.extended_id;
    rtr = frame.rtr;

    buf[TXBCTRL_OFF] = INSTRUCTION_LOAD_TXB(txb);
    buf[TXBSIDH_OFF] = sid >> SIDH_SHIFT;
    buf[TXBSIDL_OFF] = ((sid & SIDL_SID_MASK) << SIDL_SID_SHIFT) |
                       (exide << SIDL_EXIDE_SHIFT) |
                       ((eid >> SIDL_EID_SHIFT) & SIDL_EID_MASK);
    buf[TXBEID8_OFF] = (eid & 0xff00) >> 8;
    buf[TXBEID0_OFF] = (eid & 0x00ff);
    buf[TXBDLC_OFF] = (rtr << DLC_RTR_SHIFT) | frame.dlc;
    memcpy(buf + TXBDAT_OFF, frame.data, frame.dlc);
    mcp251x_hw_tx_frame(priv, buf, frame.dlc, txb);

    /* use INSTRUCTION_RTS, to avoid "repeated frame problem" */
    priv->spi_tx_buf[0] = INSTRUCTION_RTS(1 << txb);
    mcp251x_spi_write(priv, 1);
}

int mcp251x_start_tx(Priv *priv, CAN_Frame &frame)
{
    if (priv->tx_busy)
    {
        return CAN_BUSY;
    }

    // wait for transmit slot
    mutex_enter_blocking(&priv->mcp_lock);

    if (frame.dlc > 8)
    {
        frame.dlc = 0;
    }

    mcp251x_hw_tx(priv, frame, 1);
    priv->tx_busy = true;
    mutex_exit(&priv->mcp_lock);
    // Simulate interrupt, receive all messages ASAP
    // If CANINTF is empty, nothing will happen
    int last_status = platform_status_last;
    // mcp251x_can_isr(priv);
    platform_set_status(last_status);
    return 0;
}

static void mcp251x_can_isr(void *device)
{
    platform_set_status(0x66);
    Priv *priv = reinterpret_cast<Priv *>(device);

    if (mutex_try_enter(&priv->mcp_lock, nullptr))
    {
        platform_set_status(0x69);
        while (true)
        {
            u8 intf, eflag;
            intf = mcp251x_read_reg(priv, CANINTF);
            mcp251x_write_reg(priv, CANINTF, 0);
            eflag = mcp251x_read_reg(priv, EFLG);

            // first, check for messages
            if (intf & CANINTF_RX0IF)
            {
                // message in the first buffer
                mcp251x_hw_rx(priv, 0);

                // reset INTF
                mcp251x_write_bits(priv, CANINTF, CANINTF_RX0IF, 0);

                if (!(intf & CANINTF_RX1IF))
                {
                    // reread intf, rx1 could be full now
                    u8 intf1, eflag1;
                    // TODO: Combine into single operation
                    intf1 = mcp251x_read_reg(priv, CANINTF);
                    eflag1 = mcp251x_read_reg(priv, EFLG);

                    intf |= intf1;
                    eflag1 |= eflag1;
                }
            }

            if (intf & CANINTF_RX1IF)
            {
                mcp251x_hw_rx(priv, 1);

                // reset INTF
                mcp251x_write_bits(priv, CANINTF, CANINTF_RX1IF, 0);
                intf ^= CANINTF_RX1IF; // toggle RX1IF off
            }

            // error handling
            intf &= CANINTF_TX | CANINTF_ERR;

            if (intf & (CANINTF_TX | CANINTF_ERR))
            {
                // clear tx and err interrupts
                mcp251x_write_bits(priv, CANINTF, (CANINTF_TX | CANINTF_ERR), 0);
            }
            if (eflag & (EFLG_RX0OVR | EFLG_RX1OVR))
            {
                // clear error state
                mcp251x_write_bits(priv, EFLG, eflag, 0);
            }

            // TODO: Handle errors

            // Handle TX
            if (!intf)
            {
                // nothing to handle anymore
                break;
            }

            if (intf & CANINTF_TX)
            {
                priv->tx_busy = false;
                platform_set_status(0x68);
            }
            mcp251x_write_reg(priv, CANINTF, 0);
            break;
        }

        mutex_exit(&priv->mcp_lock);
    }
    else
    {
        // we cannot handle the rx frame, the transmit function will handle it later
        platform_set_status(0x66);
    }
}

static int mcp251x_set_bittiming(Priv *priv, int clock_freq, int baudrate)
{
    const u8 *cnf = nullptr;
    for (unsigned int i = 0; i < (sizeof(mcp251x_cnf_mapper)) / sizeof(mcp251x_cnf_mapper[0]); i++)
    {
        if (mcp251x_cnf_mapper[i].clockFrequency == clock_freq && mcp251x_cnf_mapper[i].baudRate == baudrate)
        {
            cnf = mcp251x_cnf_mapper[i].cnf;
            break;
        }
    }

    if (!cnf)
    {
        return STATUS_CAN_INVALIDSPEED;
    }
    mcp251x_write_reg(priv, CNF1, cnf[0]);
    mcp251x_write_reg(priv, CNF2, cnf[1]);
    mcp251x_write_reg(priv, CNF3, cnf[2]);

    return 0;
}

static int mcp251x_set_normal_mode(Priv *priv)
{
    // enable all interrupts
    mcp251x_write_reg(priv, CANINTE, CANINTE_ERRIE | CANINTE_TX2IE | CANINTE_TX1IE | CANINTE_TX0IE | CANINTE_RX1IE | CANINTE_RX0IE);

    /* Put device into normal mode */
    mcp251x_write_reg(priv, CANCTRL, CANCTRL_REQOP_NORMAL);

    // set normalmode
    u8 value;
    int ret;
    platform_set_status(STATUS_CAN_MODESETWAIT); // signal waiting for can reset, we cannot proceed without CAN
    ret = read_poll_timeout_blocking(mcp251x_read_stat, value, value == CANCTRL_REQOP_NORMAL, 1000, 10, priv);
    if (ret)
    {
        return ret;
    }

    return 0;
}

Priv *mcp251x_platform_init(int clock_freq, int baudrate)
{
    _gpio_init(PLATFORM_PIN_CAN_CS);
    gpio_set_dir(PLATFORM_PIN_CAN_CS, GPIO_OUT);
    gpio_put(PLATFORM_PIN_CAN_CS, 1); // CS is active low
    pinMode(PLATFORM_PIN_CAN_INT, INPUT);

    Priv *instance = new Priv;
    instance->tx_busy = false;
    mutex_init(&instance->mcp_lock);
    queue_init(&instance->can_rx_queue, sizeof(CAN_Frame), CAN_RX_QUEUE_LEN);
    memset(instance->spi_tx_buf, 0, SPI_TRANSFER_BUF_LEN);
    memset(instance->spi_rx_buf, 0, SPI_TRANSFER_BUF_LEN);

    mutex_enter_blocking(&instance->mcp_lock);

    attachInterruptParam(PLATFORM_PIN_CAN_INT, mcp251x_can_isr, PinStatus::FALLING, instance);

    // reset device

    int ret = mcp251x_reset(instance);
    if (ret)
    {
        platform_set_status(ret);
        goto retnull;
    }

    ret = mcp251x_set_bittiming(instance, clock_freq, baudrate);
    if (ret)
    {
        platform_set_status(ret);
        goto retnull;
    }

    ret = mcp251x_set_normal_mode(instance);
    if (ret)
    {
        platform_set_status(ret);
        goto retnull;
    }

    mcp251x_write_reg(instance, RXBCTRL(0), RXBCTRL_BUKT | RXBCTRL_RXM0 | RXBCTRL_RXM1);
    mcp251x_write_reg(instance, RXBCTRL(1), RXBCTRL_RXM0 | RXBCTRL_RXM1);
    mcp251x_write_reg(instance, CANINTF, 0);

    // Skip null return
    mutex_exit(&instance->mcp_lock);
    goto normalret;

retnull:
    mutex_exit(&instance->mcp_lock);
    delete instance;
    instance = nullptr;
    detachInterrupt(PLATFORM_PIN_CAN_INT);

normalret:
    //
    return instance;
}
