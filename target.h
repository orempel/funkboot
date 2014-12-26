#ifndef __TARGET_H__
#define __TARGET_H__

/* *********************************************************************** */
#if defined(CONFIG_funkstuff168)
/*
 * using ATmega168 @16MHz:
 * Fuse E: 0xFA (512 words bootloader)
 * Fuse H: 0xDD (2.7V BOD)
 * Fuse L: 0xFF (external crystal)
 */
#define F_CPU               16000000
#define RFM12_ADDRESS       0x11
#define TIMEOUT             1000

/* 1ms @16MHz */
#define TIMER_RELOAD        (0xFF - 250)

#define VERSION_STRING      "FUNKBOOTm168v1.0"
#define SIGNATURE_BYTES     0x1E, 0x94, 0x06

#define LED_INIT()          { DDRD |= ((1<<PORTD5) | (1<<PORTD6)); LED_OFF(); }
#define LED_GN_ON()         PORTD &= ~(1<<PORTD5)
#define LED_GN_OFF()        PORTD |= (1<<PORTD5)
#define LED_GN_TOGGLE()     PORTD ^= (1<<PORTD5)
#define LED_RT_ON()         PORTD &= ~(1<<PORTD6)
#define LED_RT_OFF()        PORTD |= (1<<PORTD6)
#define LED_OFF()           PORTD |= ((1<<PORTD5) | (1<<PORTD6))


#define RFM12_INT_INIT()    EICRA |= (1<<ISC11)
#define RFM12_INT_ON()      EIMSK |= (1<<INT1)
#define RFM12_INT_OFF()     EIMSK &= ~(1<<INT1)
#define RFM12_INT_CLEAR()   EIFR = INTF1
#define RFM12_INT_VECT      INT1_vect

#define RFM12_CS_INIT()     DDRD |= (1<<PORTD7)
#define RFM12_CS_ACTIVE()   PORTD &= ~(1<<PORTD7)
#define RFM12_CS_INACTIVE() PORTD |= (1<<PORTD7)

#define RFM12_SPI_INIT()    {   /* SS, SCK and MOSI are outputs */ \
                                DDRB |= (1<<PORTB2) | (1<<PORTB3) | (1<<PORTB5); \
                                /* SPI Master, F_CPU /16 */ \
                                SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0); \
                            }

/* *********************************************************************** */
#else
#error "unknown CONFIG"
#endif
/* *********************************************************************** */

#endif /* __TARGET_H__ */
