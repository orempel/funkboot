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
#include <avr/boot.h>
#include <avr/pgmspace.h>

#include <string.h>

#include "target.h"
#include "rfm12.h"

/* *********************************************************************** */

#define BOOTWAIT_EXPIRED                0x0000
#define BOOTWAIT_INTERRUPTED            0xFFFF

#define MSG_TYPE_REQUEST                0x00    /* master -> slave req */
#define MSG_TYPE_CONFIRMATION           0x40    /* master -> slave rsp */
#define MSG_TYPE_INDICATION             0x80    /* slave -> master req */
#define MSG_TYPE_RESPONSE               0xC0    /* slave -> master rsp */
#define MSG_TYPE_MASK                   0xC0
#define MSG_CMD_MASK                    0x3F

#define MSG_CMD_SWITCHAPP_REQUEST       (MSG_TYPE_REQUEST       | 0x20)
#define MSG_CMD_SWITCHAPP_RESPONSE      (MSG_TYPE_RESPONSE      | 0x20)

#define MSG_CMD_VERSION_REQUEST         (MSG_TYPE_REQUEST       | 0x21)
#define MSG_CMD_VERSION_RESPONSE        (MSG_TYPE_RESPONSE      | 0x21)

#define MSG_CMD_CHIPINFO_REQUEST        (MSG_TYPE_REQUEST       | 0x22)
#define MSG_CMD_CHIPINFO_RESPONSE       (MSG_TYPE_RESPONSE      | 0x22)

#define MSG_CMD_READ_REQUEST            (MSG_TYPE_REQUEST       | 0x23)
#define MSG_CMD_READ_RESPONSE           (MSG_TYPE_RESPONSE      | 0x23)

#define MSG_CMD_WRITE_REQUEST           (MSG_TYPE_REQUEST       | 0x24)
#define MSG_CMD_WRITE_RESPONSE          (MSG_TYPE_RESPONSE      | 0x24)

#define CAUSE_SUCCESS                   0x00
#define CAUSE_NOT_SUPPORTED             0xF0
#define CAUSE_INVALID_PARAMETER         0xF1
#define CAUSE_UNSPECIFIED_ERROR         0xFF

#define BOOTMODE_BOOTLOADER             0x00
#define BOOTMODE_APPLICATION            0x80

#define MEMTYPE_FLASH                   0x01
#define MEMTYPE_EEPROM                  0x02

/* *********************************************************************** */

struct bootloader_msg
{
    uint8_t command;
    uint8_t seqnum;
    uint8_t cause;

    union {
        struct {
            uint8_t     app;
        } switchapp;

        struct {
            uint8_t     data[16];
        } version;

        struct {
            uint8_t     data[8];
        } chipinfo;

        struct {
            uint16_t    address;
            uint8_t     mem_type;
            uint8_t     size;
        } read_req;

        struct {
            uint8_t     data[32];
        } read_rsp;

        struct {
            uint16_t    address;
            uint8_t     mem_type;
            uint8_t     size;
            uint8_t     data[32];
        } write_req;
    } p;
};

/* *********************************************************************** */

#define LED_RX 0
#define LED_TX 1

static uint8_t led_timer[2];
volatile static uint8_t clock_tick;

const static uint8_t version_info[16] = VERSION_STRING;
const static uint8_t chip_info[8] = {
    SIGNATURE_BYTES,

    SPM_PAGESIZE,

    ((BOOTLOADER_START) >> 8) & 0xFF,
    (BOOTLOADER_START) & 0xFF,
    ((E2END +1) >> 8 & 0xFF),
    (E2END +1) & 0xFF
};


/* *********************************************************************** */


static void read_flash(uint8_t *data, uint16_t address, uint8_t size)
{
    while (size--)
    {
        *data++ = pgm_read_byte_near(address++);
    }
} /* read_flash_byte */


static void write_flash(uint8_t *data, uint16_t address, uint8_t size)
{
    static uint16_t pagestart;
    static uint8_t pagesize;

    if ((address & (SPM_PAGESIZE -1)) == 0x00)
    {
        pagestart = address;
        pagesize = SPM_PAGESIZE;

        boot_page_erase(pagestart);
        boot_spm_busy_wait();
    }

    while (size && pagesize)
    {
        uint16_t dataword;

        dataword = (*data++);
        dataword |= (*data++) << 8;
        boot_page_fill(address, dataword);

        address += 2;
        pagesize -= 2;
        size -= 2;
    }

    if (pagesize == 0)
    {
        boot_page_write(pagestart);
        boot_spm_busy_wait();
        boot_rww_enable();
    }
} /* write_flash */


static void read_eeprom(uint8_t *data, uint16_t address, uint8_t size)
{
    while (size--)
    {
        EEARL = address;
        EEARH = (address >> 8);
        EECR |= (1<<EERE);

        address++;
        *data++ = EEDR;
    }
} /* read_eeprom */


static void write_eeprom(uint8_t *data, uint16_t address, uint8_t size)
{
    while (size--)
    {
        EEARL = address;
        EEARH = (address >> 8);
        EEDR = *data++;

        address++;

        cli();
#if defined (__AVR_ATmega168__)
        EECR |= (1<<EEMPE);
        EECR |= (1<<EEPE);
#endif
        sei();

        eeprom_busy_wait();
    }
} /* write_eeprom */


ISR(TIMER0_OVF_vect)
{
    TCNT0 = TIMER_RELOAD;

    clock_tick = 1;
} /* TIMER0_OVF_vect */


static void (*jump_to_app)(void) __attribute__ ((noreturn)) = 0x0000;


#if defined(__AVR_ATmega168__)
/*
 * For newer devices the watchdog timer remains active even after a
 * system reset. So disable it as soon as possible.
 * automagically called on startup
 */
void disable_wdt_timer(void) __attribute__((naked, section(".init3")));
void disable_wdt_timer(void)
{
    MCUSR = 0;
    WDTCSR = (1<<WDCE) | (1<<WDE);
    WDTCSR = (0<<WDE);
} /* disable_wdt_timer */
#endif


int main(void) __attribute__ ((noreturn));
int main(void)
{
    /* init LEDs */
    LED_INIT();

#if defined (__AVR_ATmega168__)
    /* move interrupt-vectors to bootloader */
    MCUCR = (1<<IVCE);
    MCUCR = (1<<IVSEL);

    /* timer0, FCPU/64, overflow interrupt */
    TCCR0B = (1<<CS01) | (1<<CS00);
    TIMSK0 = (1<<TOIE0);
#endif

    rfm12_init(RFM12_ADDRESS);

    sei();

    uint16_t boot_timeout = TIMEOUT;
    while (1)
    {
        if (clock_tick == 0)
            continue;

        clock_tick = 0;

        if (led_timer[LED_RX] > 0)
        {
            led_timer[LED_RX]--;
            LED_GN_ON();
        }
        else
        {
            LED_GN_OFF();
        }

        if (led_timer[LED_TX] > 0)
        {
            led_timer[LED_TX]--;
            LED_RT_ON();
        }
        else
        {
            LED_RT_OFF();
        }

        /* do periodic work (wait for 5 ticks silence before start TX) */
        rfm12_tick(5);

        /* get TX buffer */
        struct rfm12_packet *rsp_pkt = rfm12_get_txpkt();

        if (boot_timeout == BOOTWAIT_EXPIRED)
        {
            /* timeout elapsed and TX buffer available (tx completed) -> start application */
            if (rsp_pkt != NULL)
            {
                break;
            }
        }
        else if (boot_timeout != BOOTWAIT_INTERRUPTED)
        {
            boot_timeout--;
        }

        if ((boot_timeout & 0x3F) == 0)
        {
            led_timer[LED_TX] = 5;
        }

        /* get RX data */
        struct rfm12_packet *req_pkt = rfm12_get_rxpkt();
        if (req_pkt == NULL)
        {
            /* no data available */
            continue;
        }
        else
        {
            led_timer[LED_RX] = 5;

            /* no tx buffer available, ignore request */
            if (rsp_pkt == NULL)
            {
                rfm12_clear_rx();
                continue;
            }
        }

        /* stay in bootloader */
        boot_timeout = BOOTWAIT_INTERRUPTED;

        struct bootloader_msg *req_msg = (struct bootloader_msg *)req_pkt->data;
        struct bootloader_msg *rsp_msg = (struct bootloader_msg *)rsp_pkt->data;

        /* retransmitted request -> retransmit response */
        if ((req_pkt->source_address == rsp_pkt->dest_address) &&
            ((req_msg->command & MSG_CMD_MASK) == (rsp_msg->command & MSG_CMD_MASK)) &&
            (req_msg->seqnum == rsp_msg->seqnum)
           )
        {
            /* RX packet no longer needed */
            rfm12_clear_rx();

            /* transmit response */
            if (rfm12_start_tx())
            {
                led_timer[LED_TX] = 5;
            }

            continue;
        }

        rsp_pkt->dest_address   = req_pkt->source_address;
        rsp_pkt->data_length    = 3;
        rsp_msg->command        = req_msg->command | MSG_TYPE_RESPONSE;
        rsp_msg->seqnum         = req_msg->seqnum;
        rsp_msg->cause          = CAUSE_SUCCESS;

        switch (req_msg->command)
        {
            case MSG_CMD_SWITCHAPP_REQUEST:
                if (req_msg->p.switchapp.app == BOOTMODE_APPLICATION)
                {
                    boot_timeout = BOOTWAIT_EXPIRED;
                }
                else if (req_msg->p.switchapp.app != BOOTMODE_BOOTLOADER)
                {
                    rsp_msg->cause = CAUSE_INVALID_PARAMETER;
                }
                break;

            case MSG_CMD_VERSION_REQUEST:
                memcpy(rsp_msg->p.version.data, version_info, sizeof(rsp_msg->p.version.data));
                rsp_pkt->data_length += sizeof(rsp_msg->p.version.data);
                break;

            case MSG_CMD_CHIPINFO_REQUEST:
                memcpy(rsp_msg->p.chipinfo.data, chip_info, sizeof(rsp_msg->p.chipinfo.data));
                rsp_pkt->data_length += sizeof(rsp_msg->p.chipinfo.data);
                break;

            case MSG_CMD_READ_REQUEST:
                if (req_msg->p.read_req.mem_type == MEMTYPE_FLASH)
                {
                    read_flash(rsp_msg->p.read_rsp.data,
                            req_msg->p.read_req.address,
                            req_msg->p.read_req.size);

                    rsp_pkt->data_length += req_msg->p.read_req.size;
                }
                else if (req_msg->p.read_req.mem_type == MEMTYPE_EEPROM)
                {
                    read_eeprom(rsp_msg->p.read_rsp.data,
                                req_msg->p.read_req.address,
                                req_msg->p.read_req.size);

                    rsp_pkt->data_length += req_msg->p.read_req.size;
                }
                else
                {
                    rsp_msg->cause = CAUSE_INVALID_PARAMETER;
                }
                break;

            case MSG_CMD_WRITE_REQUEST:
                if (req_msg->p.write_req.mem_type == MEMTYPE_FLASH)
                {
                    write_flash(req_msg->p.write_req.data,
                                req_msg->p.write_req.address,
                                req_msg->p.write_req.size);
                }
                else if (req_msg->p.write_req.mem_type == MEMTYPE_EEPROM)
                {
                    write_eeprom(req_msg->p.write_req.data,
                                 req_msg->p.write_req.address,
                                 req_msg->p.write_req.size);
                }
                else
                {
                    rsp_msg->cause = CAUSE_INVALID_PARAMETER;
                }
                break;

            default:
                rsp_msg->cause  = CAUSE_NOT_SUPPORTED;
                break;
        }

        /* RX packet no longer needed */
        rfm12_clear_rx();

        /* transmit response */
        if (rfm12_start_tx())
        {
            led_timer[LED_TX] = 5;
        }
    }

    cli();

#if defined (__AVR_ATmega168__)
    /* disable timer0 */
    TIMSK0 = 0x00;
    TCCR0B = 0x00;

    /* move interrupt vectors back to application */
    MCUCR = (1<<IVCE);
    MCUCR = (0<<IVSEL);
#endif

    LED_OFF();

    jump_to_app();
} /* main */
