/*
 * This driver is based on the mcp251x.c driver from Linux and uses raw pico-sdk
 * SPI access.
 *
 * This is both an SPI and a CAN driver
 */
#include "driver/can/spi_mcp2510.h"
#include <Arduino.h>

#include "driver/can/can_timing.h"
#include "driver/can/canstat.h"

#include "platform/platform.h"

#include <hardware/gpio.h>
#include <pico/mutex.h>

static const bool DEBUG_REGS = false;

static void mcp251x_can_isr(void *device);

static PlatformStatus mcp251x_spi_write(Priv *priv, size_t len) {
    digitalWrite(PLATFORM_PIN_CAN_CS, LOW);
    spi_write_blocking(spi0, priv->spi_tx_buf, len);
    digitalWrite(PLATFORM_PIN_CAN_CS, HIGH);
    return PlatformStatus::STATUS_OK;
}

static PlatformStatus mcp251x_spi_transfer(Priv *priv, size_t len) {
    digitalWrite(PLATFORM_PIN_CAN_CS, LOW);
    spi_write_read_blocking(spi0, priv->spi_tx_buf, priv->spi_rx_buf, len);
    digitalWrite(PLATFORM_PIN_CAN_CS, HIGH);
    return PlatformStatus::STATUS_OK;
}

static int mcp251x_read_reg(Priv *priv, u8 reg) {

    priv->spi_tx_buf[0] = INSTRUCTION_READ;
    priv->spi_tx_buf[1] = reg;
    mcp251x_spi_transfer(priv, 3);
    if (!priv->in_irq) {
        LOGF_IF(DEBUG_REGS, "mcp251x_read_reg: reading %x = %x", reg,
                priv->spi_rx_buf[2]);
    }
    return priv->spi_rx_buf[2];
}

static void mcp251x_write_reg(Priv *priv, u8 reg, u8 value) {
    if (!priv->in_irq) {
        LOGF_IF(DEBUG_REGS, "mcp251x_write_reg: writing %x = %x", reg, value);
    }
    priv->spi_tx_buf[0] = INSTRUCTION_WRITE;
    priv->spi_tx_buf[1] = reg;
    priv->spi_tx_buf[2] = value;

    mcp251x_spi_write(priv, 3);
}

static void mcp251x_write_bits(Priv *priv, u8 reg, u8 mask, u8 value) {
    priv->spi_tx_buf[0] = INSTRUCTION_BIT_MODIFY;
    priv->spi_tx_buf[1] = reg;
    priv->spi_tx_buf[2] = mask;
    priv->spi_tx_buf[3] = value;

    mcp251x_spi_write(priv, 4);
}

static u8 mcp251x_read_stat(Priv *priv) {
    return mcp251x_read_reg(priv, CANSTAT) & CANCTRL_REQOP_MASK;
}

static PlatformStatus mcp251x_reset(Priv *priv) {
    delay(5); // Wait for oscillator startup

    // send reset
    priv->spi_tx_buf[0] = INSTRUCTION_RESET;
    PlatformStatus ret = mcp251x_spi_write(priv, 1);
    if (is_err(ret)) {
        return ret;
    }
    delay(5); // Wait for oscillator reset

    // wait for reset
    platform_set_status(
        PlatformStatus::STATUS_DRIVER_CAN_RESET_WAIT); // signal waiting for can
                                                       // reset, we cannot
                                                       // proceed without CAN
    u8 value = 0;

    ret = read_poll_timeout_blocking(
        mcp251x_read_stat, value, value == CANCTRL_REQOP_CONF, 1000, 10, priv);
    if (is_err(ret)) {
        return ret;
    }
    return PlatformStatus::STATUS_OK;
}

#define can_cc_dlc2len(dlc) (min((dlc), 8))

static void mcp251x_hw_rx_frame(Priv *priv, u8 *buf, int rxb) {
    int i, len;

    for (i = 1; i < RXBDAT_OFF; i++)
        buf[i] = mcp251x_read_reg(priv, RXBCTRL(rxb) + i);

    len = can_cc_dlc2len(buf[RXBDLC_OFF] & RXBDLC_LEN_MASK);
    for (; i < (RXBDAT_OFF + len); i++)
        buf[i] = mcp251x_read_reg(priv, RXBCTRL(rxb) + i);
}

static void mcp251x_hw_rx(Priv *priv, int rxb) {
    CAN_Frame skb{};
    u8 buf[SPI_TRANSFER_BUF_LEN];

    mcp251x_hw_rx_frame(priv, buf, rxb);

    if (buf[RXBSIDL_OFF] & RXBSIDL_IDE) {
        // we got an extended frame
        skb.extended_id = 0;
        skb.extended_id |= (buf[RXBSIDL_OFF] & RXBSIDL_EID) << 16;
        skb.extended_id |= (buf[RXBEID8_OFF]) << 8;
        skb.extended_id |= (buf[RXBEID0_OFF]);
        skb.extended = true;
        if (buf[RXBDLC_OFF] & RXBDLC_RTR) {
            skb.rtr = true;
        }
    } else {
        // standard frame
        skb.extended = false;
        skb.standard_id = 0;
        skb.standard_id |= (buf[RXBSIDH_OFF] << RXBSIDH_SHIFT) |
                           (buf[RXBSIDL_OFF] >> RXBSIDL_SHIFT);
        if (buf[RXBSIDL_OFF] & RXBSIDL_SRR) {
            skb.rtr = true;
        }
    }
    skb.dlc = can_cc_dlc2len(buf[RXBDLC_OFF] & RXBDLC_LEN_MASK);
    if (!(skb.rtr)) {
        memcpy(skb.data, buf + RXBDAT_OFF, skb.dlc);
    }
    g_can_stat.rx_byte_count += skb.dlc;
    if (priv->can_rx_queue.add(skb, true)) {
        g_can_stat.rx_received_frame_count++;
    } else {
        g_can_stat.rx_dropped_frame_count++;
    }
}

static void mcp251x_hw_tx_frame(Priv *priv, u8 *buf, int len, int txb) {
    memcpy(priv->spi_tx_buf + 1, buf, SPI_TRANSFER_BUF_LEN);
    priv->spi_tx_buf[0] = INSTRUCTION_WRITE;
    priv->spi_tx_buf[1] = TXBCTRL(txb) + TXBSIDH_OFF;

    mcp251x_spi_write(priv, TXBDAT_OFF + len);
}

static void mcp251x_hw_tx(Priv *priv, const CAN_Frame &frame, int txb) {
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
    g_can_stat.tx_byte_count += frame.dlc;
    /* use INSTRUCTION_RTS, to avoid "repeated frame problem" */
    priv->spi_tx_buf[0] = INSTRUCTION_RTS(1 << txb);
    mcp251x_spi_write(priv, 1);
}

int mcp251x_start_tx(Priv *priv, CAN_Frame &frame) {
    // wait for transmit slot
    if (frame.dlc > 8) {
        frame.dlc = 8;
    }
    g_can_stat.tx_req_byte_count += frame.dlc;
    if (priv->tx1_busy && priv->tx0_busy) {
        if (priv->can_tx_queue.insert(frame)) {
            g_can_stat.tx_queued_frame_count++;
            LOG("frame queued");
            return 0;
        } else {
            g_can_stat.tx_dropped_frame_count++;
            mcp251x_can_isr(priv);
        }
        return 0;
    }

    int txb = priv->tx1_busy ? 0 : 1;

    mutex_enter_blocking(&priv->mcp_lock);
    mcp251x_hw_tx(priv, frame, txb);
    if (txb == 1) {
        priv->tx1_busy = true;
    } else {
        priv->tx0_busy = true;
    }
    mutex_exit(&priv->mcp_lock);
    mcp251x_can_isr(priv);
    return 0;
}

static void mcp251x_can_isr(void *device) {
    uint32_t INTERR = save_and_disable_interrupts();
    g_can_stat.irq_handled++;
    Priv *priv = reinterpret_cast<Priv *>(device);

    IRQ_LOG("CAN interrupt happened");
    if (mutex_try_enter(&priv->mcp_lock, nullptr)) {
        priv->in_irq = true;
        while (true) {
            u8 intf, eflag;

            intf = mcp251x_read_reg(priv, CANINTF);
            IRQ_LOGF("interrupt flag: %x", intf);
            eflag = mcp251x_read_reg(priv, EFLG);
            IRQ_LOGF("error flag: %x", eflag);
            // first, check for messages
            if (intf & CANINTF_RX0IF) {
                // message in the first buffer
                mcp251x_hw_rx(priv, 0);

                // reset INTF
                mcp251x_write_bits(priv, CANINTF, CANINTF_RX0IF, 0);
                intf ^= CANINTF_RX0IF;
                if (!(intf & CANINTF_RX1IF)) {
                    // reread intf, rx1 could be full now
                    u8 intf1, eflag1;
                    // TODO: Combine into single operation
                    intf1 = mcp251x_read_reg(priv, CANINTF);
                    eflag1 = mcp251x_read_reg(priv, EFLG);

                    intf |= intf1;
                    eflag |= eflag1;
                }
            }

            if (intf & CANINTF_RX1IF) {
                mcp251x_hw_rx(priv, 1);

                // reset INTF
                mcp251x_write_bits(priv, CANINTF, CANINTF_RX1IF, 0);
                intf ^= CANINTF_RX1IF; // toggle RX1IF off
            }

            // error handling
            intf &= CANINTF_TX | CANINTF_ERR;

            if (intf & (CANINTF_TX | CANINTF_ERR)) {
                // clear tx and err interrupts
                mcp251x_write_bits(priv, CANINTF, (CANINTF_TX | CANINTF_ERR),
                                   0);
            }
            if (eflag & (EFLG_RX0OVR | EFLG_RX1OVR)) {
                // clear error state
                mcp251x_write_bits(priv, EFLG, eflag, 0);
            }

            // TODO: Handle errors

            // Handle TX
            if (!intf) {
                // nothing to handle anymore
                break;
            }

            if (intf & CANINTF_TX) {
                CAN_Frame frame2send;
                if (intf & CANINTF_TX0IF) {
                    g_can_stat.tx_sent_frame_count++;
                    if (priv->can_tx_queue.pull(&frame2send)) {
                        mcp251x_hw_tx(priv, frame2send, 0);
                    } else {
                        priv->tx0_busy = false;
                    }

                    if (!(intf & CANINTF_TX1IF)) {
                        // reread intf, intf could be full now
                        intf = mcp251x_read_reg(priv, CANINTF);
                    }
                }
                if (intf & CANINTF_TX1IF) {
                    g_can_stat.tx_sent_frame_count++;
                    if (priv->can_tx_queue.pull(&frame2send)) {
                        mcp251x_hw_tx(priv, frame2send, 1);
                    } else {
                        priv->tx1_busy = false;
                    }
                }
            }
            mcp251x_write_bits(priv, CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF),
                               0);
        }

        mutex_exit(&priv->mcp_lock);
        priv->in_irq = false;
    } else {
        IRQ_LOG("cannot access SPI :(");
        // we cannot handle the rx frame, the transmit function will handle
        // it later
        g_can_stat.isr_lost_race_count++;
    }
    restore_interrupts(INTERR);
}

static PlatformStatus mcp251x_set_bittiming(Priv *priv, int clock_freq,
                                            int baudrate) {
    const u8 *cnf = nullptr;
    for (unsigned int i = 0;
         i < (sizeof(mcp251x_cnf_mapper)) / sizeof(mcp251x_cnf_mapper[0]);
         i++) {
        if (mcp251x_cnf_mapper[i].clockFrequency == clock_freq &&
            mcp251x_cnf_mapper[i].baudRate == baudrate) {
            cnf = mcp251x_cnf_mapper[i].cnf;
            break;
        }
    }

    if (!cnf) {
        return PlatformStatus::STATUS_DRIVER_CAN_INVALID_SPEED;
    }
    mcp251x_write_reg(priv, CNF1, cnf[0]);
    mcp251x_write_reg(priv, CNF2, cnf[1]);
    mcp251x_write_reg(priv, CNF3, cnf[2]);

    return PlatformStatus::STATUS_OK;
}

static PlatformStatus mcp251x_set_normal_mode(Priv *priv) {
    // enable all interrupts
    mcp251x_write_reg(priv, CANINTE,
                      CANINTE_ERRIE | CANINTE_TX2IE | CANINTE_TX1IE |
                          CANINTE_TX0IE | CANINTE_RX1IE | CANINTE_RX0IE);

    /* Put device into normal mode */
    mcp251x_write_reg(priv, CANCTRL, CANCTRL_REQOP_LOOPBACK);

    // set normalmode
    u8 value = 0;
    platform_set_status(
        PlatformStatus::STATUS_DRIVER_CAN_MODESET_WAIT); // signal waiting for
                                                         // can reset, we cannot
                                                         // proceed without CAN
    TRY(read_poll_timeout_blocking(mcp251x_read_stat, value,
                                   value == CANCTRL_REQOP_LOOPBACK, 1000, 10,
                                   priv));

    return PlatformStatus::STATUS_OK;
}

Priv *mcp251x_platform_init(int clock_freq, int baudrate, Priv *instance) {
    pinMode(PLATFORM_PIN_CAN_CS, OUTPUT);
    pinMode(PLATFORM_PIN_CAN_INT, INPUT);

    spi_deinit(spi0);
    gpio_set_function(PLATFORM_PIN_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PLATFORM_PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PLATFORM_PIN_SPI_MISO, GPIO_FUNC_SPI);
    digitalWrite(PLATFORM_PIN_CAN_CS, HIGH);

    _spi_init(spi0, PLATFORM_CAN_SPI_BAUD);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    spi_set_slave(spi0, false);

    if (!mutex_is_initialized(&instance->mcp_lock)) {
        *instance = Priv{};
    }
    instance->tx1_busy = false;
    instance->tx0_busy = false;
    mutex_init(&instance->mcp_lock);
    memset(instance->spi_tx_buf, 0, SPI_TRANSFER_BUF_LEN);
    memset(instance->spi_rx_buf, 0, SPI_TRANSFER_BUF_LEN);

    mutex_enter_blocking(&instance->mcp_lock);
    // reset device

    PlatformStatus ret = mcp251x_reset(instance);
    platform_set_status(ret);
    LOGF("reset result: %d", statuscode(ret));
    if (is_err(ret)) {
        goto retnull;
    }

    ret = mcp251x_set_bittiming(instance, clock_freq, baudrate);
    platform_set_status(ret);
    LOGF("bitset result: %d", statuscode(ret));
    if (is_err(ret)) {
        goto retnull;
    }

    ret = mcp251x_set_normal_mode(instance);
    platform_set_status(ret);
    LOGF("modeset result: %d", statuscode(ret));
    if (is_err(ret)) {
        goto retnull;
    }

    mcp251x_write_reg(instance, RXBCTRL(0),
                      RXBCTRL_BUKT | RXBCTRL_RXM0 | RXBCTRL_RXM1);
    mcp251x_write_reg(instance, RXBCTRL(1), RXBCTRL_RXM0 | RXBCTRL_RXM1);
    mcp251x_write_reg(instance, CANINTF, 0);

    attachInterruptParam(PLATFORM_PIN_CAN_INT, mcp251x_can_isr,
                         PinStatus::FALLING, instance);
    LOG("attached interrupt");

    // Skip null return
    mutex_exit(&instance->mcp_lock);
    goto normalret;

retnull:
    mutex_exit(&instance->mcp_lock);
    instance = nullptr;
    detachInterrupt(PLATFORM_PIN_CAN_INT);
    platform_set_status(PlatformStatus::STATUS_DRIVER_CAN_INIT_FAILED);

normalret:
    //
    return instance;
}
