/***************************************************************************
 *   Copyright (C) 11/2014 by Olaf Rempel                                  *
 *   razzor@kopf-tisch.de                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; version 2 of the License,               *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/crc16.h>

#include "target.h"
#include "rfm12_hw.h"
#include "rfm12.h"

/* *********************************************************************** */

#define RFM12_PREAMBLE              0xAA
#define RFM12_SYNC_LSB              0xD4
#define RFM12_SYNC_MSB              0x2D

#define RFM12_BASEBAND              RFM12_BAND_868
#define RFM12_XTAL_LOAD             RFM12_XTAL_11_5PF
#define RFM12_FREQUENCY_CALC        RFM12_FREQUENCY_CALC_868
#define RFM12_FREQUENCY             869800000UL
#define RFM12_DATARATE              RFM12_DATARATE_CALC_HIGH(19200.0)
#define RFM12_FILTER_BW             RFM12_RXCTRL_BW_400
#define RFM12_LNA_GAIN              RFM12_RXCTRL_LNA_6
#define RFM12_RSSI_THRESHOLD        RFM12_RXCTRL_RSSI_79
#define RFM12_POWER                 RFM12_TXCONF_POWER_0
#define RFM12_FSK_SHIFT             125000

/* *********************************************************************** */

#define RFM12_DATA_STATE_FREE       0x00
#define RFM12_DATA_STATE_USED       0x01

struct rfm12_data
{
    uint8_t             state;          /* RFM12_DATA_STATE_* */
    struct rfm12_packet packet;
};

#define RFM12_CTX_STATE_INAVTIVE    0x00
#define RFM12_CTX_STATE_RX_IDLE     0x01
#define RFM12_CTX_STATE_RX_ACTIVE   0x02
#define RFM12_CTX_STATE_TX_ACTIVE   0x03

struct rfm12_context
{
    uint8_t             state;          /* RFM12_CTX_STATE_* */
    uint8_t             ch_free_ticks;  /* number of ticks channel is unused */
    uint8_t             own_address;

    uint8_t             rx_idx_in;      /* pkt receiving */
    uint8_t             rx_idx_out;     /* pkt given to app */
    uint8_t             rx_checksum;    /* receive checksum */
    uint8_t             rx_data_idx;    /* byte position inside rx[].packet */

    uint8_t             tx_data_idx;    /* byte position inside tx.packet */

    struct rfm12_data   rx[2];
    struct rfm12_data   tx;
};

static struct rfm12_context rfm12_ctx;

/* *********************************************************************** */

static uint16_t rfm12_data(uint16_t txdata)
{
    uint16_t retval;

    RFM12_CS_ACTIVE();

    SPDR = (txdata >> 8);
    while (!(SPSR & (1<<SPIF)));
    retval = (SPDR << 8);

    SPDR = (txdata & 0xFF);
    while (!(SPSR & (1<<SPIF)));
    retval |= SPDR;

    RFM12_CS_INACTIVE();

    return retval;
} /* rfm12_data */


ISR(RFM12_INT_VECT)
{
    uint16_t    rfm12_status;
    uint8_t     rfm12_reset_fifo = 0;
    uint8_t     data;

    /* disable interrupt */
    RFM12_INT_OFF();

    /* clear interrupt flag */
    RFM12_INT_CLEAR();

    /* read upper status byte (interrupt flags) */
    rfm12_status = rfm12_data(RFM12_CMD_STATUS);

    /* FIFO interrupt */
    if (rfm12_status & RFM12_STATUS_FFIT)
    {
        switch (rfm12_ctx.state)
        {
            case RFM12_CTX_STATE_RX_IDLE:
                data = rfm12_data(RFM12_CMD_READ);

                /* check if buffer is free */
                if (rfm12_ctx.rx[rfm12_ctx.rx_idx_in].state != RFM12_DATA_STATE_FREE)
                {
                    rfm12_reset_fifo = 1;
                    break;
                }

                /* store first header byte */
                rfm12_ctx.rx[rfm12_ctx.rx_idx_in].packet.sync[RFM12_PKT_SYNC_SIZE] = data;

                /* store state */
                rfm12_ctx.state         = RFM12_CTX_STATE_RX_ACTIVE;
                rfm12_ctx.rx_data_idx   = 1;
                rfm12_ctx.rx_checksum   = data;
                break;

            case RFM12_CTX_STATE_RX_ACTIVE:
                data = rfm12_data(RFM12_CMD_READ);

                /* check buffer size */
                if (rfm12_ctx.rx_data_idx >= (RFM12_PKT_HEADER_SIZE + RFM12_PKT_MAX_DATA_SIZE))
                {
                    rfm12_reset_fifo = 1;
                    break;
                }

                /* store data */
                (& rfm12_ctx.rx[rfm12_ctx.rx_idx_in].packet.sync[RFM12_PKT_SYNC_SIZE])[rfm12_ctx.rx_data_idx++] = data;

                /* calculate checksum */
                rfm12_ctx.rx_checksum ^= data;

                /* check if header address, data_length and checksum are ok */
                if ((rfm12_ctx.rx_data_idx == RFM12_PKT_HEADER_SIZE) &&
                    ((rfm12_ctx.rx_checksum != 0xFF) ||
                     (rfm12_ctx.rx[rfm12_ctx.rx_idx_in].packet.dest_address != rfm12_ctx.own_address) ||
                     (rfm12_ctx.rx[rfm12_ctx.rx_idx_in].packet.source_address == rfm12_ctx.own_address) ||
                     (rfm12_ctx.rx[rfm12_ctx.rx_idx_in].packet.data_length > RFM12_PKT_MAX_DATA_SIZE)
                   ))
                {
                    rfm12_reset_fifo = 1;
                    break;
                }

                /* packet complete? */
                if (rfm12_ctx.rx_data_idx == (RFM12_PKT_HEADER_SIZE + rfm12_ctx.rx[rfm12_ctx.rx_idx_in].packet.data_length))
                {
                    /* mark buffer as full */
                    rfm12_ctx.rx[rfm12_ctx.rx_idx_in].state = RFM12_DATA_STATE_USED;

                    /* switch to other buffer */
                    rfm12_ctx.rx_idx_in ^= 1;

                    /* receiving is complete, reset fifo anyway */
                    rfm12_reset_fifo = 1;
                }
                break;

            case RFM12_CTX_STATE_TX_ACTIVE:
                /* send one additional byte! (<= not <) */
                if (rfm12_ctx.tx_data_idx <= (RFM12_PKT_SYNC_SIZE + RFM12_PKT_HEADER_SIZE + rfm12_ctx.tx.packet.data_length))
                {
                    rfm12_data(RFM12_CMD_TX | rfm12_ctx.tx.packet.sync[rfm12_ctx.tx_data_idx++]);
                }
                else
                {
                    /* enable RX */
                    rfm12_data(RFM12_CMD_PWRMGT | RFM12_PWRMGT_ER | RFM12_PWRMGT_DC);

                    /* TX dummy byte to clear interrupt */
                    rfm12_data(RFM12_CMD_TX | RFM12_PREAMBLE);

                    /* mark buffer as empty */
                    rfm12_ctx.tx.state = RFM12_DATA_STATE_FREE;

                    /* transmit is complete, reset fifo */
                    rfm12_reset_fifo = 1;
                }
                break;

            default:
                rfm12_reset_fifo = 1;
                break;
        }

        if (rfm12_reset_fifo)
        {
            /* flush fifo and wait for sync pattern */
            rfm12_data(RFM12_CMD_FIFORESET | RFM12_FIFORESET_DR | (8<<4));
            rfm12_data(RFM12_CMD_FIFORESET | RFM12_FIFORESET_DR | (8<<4) | RFM12_FIFORESET_FF);

            /* wait for RX data */
            rfm12_ctx.state = RFM12_CTX_STATE_RX_IDLE;
        }
    }

    /* enable interrupt again */
    RFM12_INT_ON();
} /* INT1_vect */


void rfm12_tick(uint8_t channel_free_time)
{
    uint16_t status;

    /* receiver not idle, come back later */
    if (rfm12_ctx.state != RFM12_CTX_STATE_RX_IDLE)
    {
        return;
    }

    RFM12_INT_OFF();
    status = rfm12_data(RFM12_CMD_STATUS);
    RFM12_INT_ON();

    /* check if module receives a carrier */
    if (status & RFM12_STATUS_RSSI)
    {
        rfm12_ctx.ch_free_ticks = 0;
        return;
    }
    else if (rfm12_ctx.ch_free_ticks <= channel_free_time)
    {
        rfm12_ctx.ch_free_ticks++;
        return;
    }

    if (rfm12_ctx.tx.state == RFM12_DATA_STATE_USED)
    {
        RFM12_INT_OFF();

        /* disable receiver */
        rfm12_data(RFM12_CMD_PWRMGT | RFM12_PWRMGT_DC);

        /* put preamble in fifo */
        rfm12_data(RFM12_CMD_TX | RFM12_PREAMBLE);
        rfm12_data(RFM12_CMD_TX | RFM12_PREAMBLE);

        /* start TX */
        rfm12_data(RFM12_CMD_PWRMGT | RFM12_PWRMGT_ET | RFM12_PWRMGT_DC);

        /* change state */
        rfm12_ctx.state         = RFM12_CTX_STATE_TX_ACTIVE;
        rfm12_ctx.tx_data_idx   = 0;

        RFM12_INT_ON();
    }
} /* rfm12_tick */


static uint16_t rfm12_calc_crc(const struct rfm12_packet *pkt)
{
    uint16_t crc = 0x0000;
    uint8_t i;

    const uint8_t *tmp = pkt->data;
    for (i = 0; i < pkt->data_length; i++)
        crc = _crc_ccitt_update(crc, *tmp++);

    return crc;
} /* pkt_check_crc */


struct rfm12_packet * rfm12_get_txpkt(void)
{
    if (rfm12_ctx.tx.state != RFM12_DATA_STATE_FREE)
    {
        return (void *)0;
    }

    return &rfm12_ctx.tx.packet;
} /* rfm12_get_txpkt */


uint8_t rfm12_start_tx(void)
{
    struct rfm12_packet *pkt = &rfm12_ctx.tx.packet;

    if ((rfm12_ctx.tx.state != RFM12_DATA_STATE_FREE) &&
        (pkt->data_length > RFM12_PKT_MAX_DATA_SIZE)
       )
    {
        return 0;
    }

    /* calculate data crc */
    uint16_t *data_crc = (uint16_t *)(pkt->data + pkt->data_length);
    *data_crc = rfm12_calc_crc(pkt);
    pkt->data_length += 2;

    /* setup packet */
    pkt->sync[0]         = RFM12_SYNC_MSB;
    pkt->sync[1]         = RFM12_SYNC_LSB;
    pkt->source_address  = rfm12_ctx.own_address;
    pkt->header_checksum = pkt->dest_address ^ pkt->source_address ^ pkt->data_length ^ 0xFF;

    /* change state */
    rfm12_ctx.tx.state = RFM12_DATA_STATE_USED;

    return 1;
} /* rfm12_start_tx */


struct rfm12_packet * rfm12_get_rxpkt(void)
{
    if (rfm12_ctx.rx[rfm12_ctx.rx_idx_out].state != RFM12_DATA_STATE_USED)
    {
        return (void *)0;
    }

    /* calculate data crc */
    struct rfm12_packet *pkt        = &rfm12_ctx.rx[rfm12_ctx.rx_idx_out].packet;

    pkt->data_length -= 2;

    uint16_t            *data_crc   = (uint16_t *)(pkt->data + pkt->data_length);

    if (*data_crc != rfm12_calc_crc(pkt))
    {
        rfm12_clear_rx();
        return (void *)0;
    }

    return pkt;
} /* rfm12_get_rxpkt */


void rfm12_clear_rx(void)
{
    /* mark buffer as empty */
    rfm12_ctx.rx[rfm12_ctx.rx_idx_out].state = RFM12_DATA_STATE_FREE;

    /* switch to other buffer */
    rfm12_ctx.rx_idx_out ^= 1;
} /* rfm12_clear_rx */


static const uint16_t init_cmds[] PROGMEM =
{
    /* set power default state (disable clock output) */
    (RFM12_CMD_PWRMGT | RFM12_PWRMGT_DC),

    /* dummy write after power management change, prevent lockup of module */
    (RFM12_CMD_TX),

    /* enable internal data register and fifo, setup selected band */
    (RFM12_CMD_CFG | RFM12_CFG_EL | RFM12_CFG_EF | RFM12_BASEBAND | RFM12_XTAL_LOAD),

    /* set frequency */
    (RFM12_CMD_FREQUENCY | RFM12_FREQUENCY_CALC(RFM12_FREQUENCY)),

    /* set data rate */
    (RFM12_CMD_DATARATE | RFM12_DATARATE),

    /* set rx parameters: vdi-out, bandwidth, LNA, RSSI */
    (RFM12_CMD_RXCTRL | RFM12_RXCTRL_P16_VDI | RFM12_RXCTRL_VDI_FAST | RFM12_FILTER_BW | RFM12_LNA_GAIN | RFM12_RSSI_THRESHOLD),

    /* automatic clock lock control, digital Filter,
     * Data quality detector value 3, slow clock recovery lock
     */
    (RFM12_CMD_DATAFILTER | RFM12_DATAFILTER_AL | 3),

    /* 2 Byte Sync Pattern, Start fifo fill when sychron pattern received,
     * disable sensitive reset, Fifo filled interrupt at 8 bits
     */
    (RFM12_CMD_FIFORESET | RFM12_FIFORESET_DR | (8<<4)),

    /* set AFC to automatic, (+4 or -3)*2.5kHz Limit, fine mode, active and enabled */
    (RFM12_CMD_AFC | RFM12_AFC_AUTO_KEEP | RFM12_AFC_LIMIT_4 | RFM12_AFC_FI | RFM12_AFC_OE | RFM12_AFC_EN),

    /* set TX Power, frequency shift */
    (RFM12_CMD_TXCONF | RFM12_POWER | RFM12_TXCONF_FS_CALC(RFM12_FSK_SHIFT)),

    /* disable low dutycycle mode */
    (RFM12_CMD_DUTYCYCLE),

    /* disable wakeup timer */
    (RFM12_CMD_WAKEUP),

    /* enable rf receiver chain */
    (RFM12_CMD_PWRMGT | RFM12_PWRMGT_ER | RFM12_PWRMGT_DC),

    /* flush fifo, start receiving */
    (RFM12_CMD_FIFORESET | RFM12_FIFORESET_DR | (8<<4)),
    (RFM12_CMD_FIFORESET | RFM12_FIFORESET_DR | (8<<4) | RFM12_FIFORESET_FF),
};


void rfm12_init(uint8_t own_address)
{
    uint8_t i;

    /* init chipselect GPIO */
    RFM12_CS_INIT();
    RFM12_CS_INACTIVE();

    /* init internal SPI */
    RFM12_SPI_INIT();

    /* send init commands */
    for (i = 0; i < ( sizeof(init_cmds) / 2) ; i++)
    {
        rfm12_data(pgm_read_word(&init_cmds[i]));
    }

    /* store own address */
    rfm12_ctx.own_address   = own_address;
    rfm12_ctx.state         = RFM12_CTX_STATE_RX_IDLE;

    /* initalize & activate interrupt */
    RFM12_INT_INIT();
    RFM12_INT_CLEAR();
    RFM12_INT_ON();
} /* rfm12_init */
