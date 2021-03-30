/**
 *  Header file for OneWire functionality.
 * 
 *  Adapted taken from Arduino:
 *  -> https://hacksterio.s3.amazonaws.com/uploads/attachments/229743/OneWire.zip
 *  In combination with:
 *  -> https://github.com/szszoke/atmega328p/blob/master/onewire/
 */

#ifndef _ONE_WIRE_H_
#define _ONE_WIRE_H_

/***** INCLUDES ***************************************************************/
#include <stdio.h>
#include <stdint.h>


/***** MACROS *****************************************************************/
/*** Enable/Disable Functionality ***/
/* Exclude onewire_search by setting to (0) */
#define ONEWIRE_SEARCH                  (1)
/* Exclude CRC checks by setting to (0) */
#define ONEWIRE_CRC                     (1)
/* Select table-lookup for 8-bit CRC (1)
 * or use the comparably slower algorithm (0) */
#define ONEWIRE_CRC8_TABLE              (1)
/* Allow 16-bit CRC checks by setting to (1) */
#define ONEWIRE_CRC16                   (1)

/*** GPIO access ***/
#define ONEWIRE_GPIO_INPUT(gpio)        (*(gpio->ddr) &= ~_BV(gpio->portpin))
#define ONEWIRE_GPIO_OUTPUT(gpio)       (*(gpio->ddr) |= _BV(gpio->portpin))
#define ONEWIRE_GPIO_LOW(gpio)          (*(gpio->port) &= ~_BV(gpio->portpin))
#define ONEWIRE_GPIO_HIGH(gpio)         (*(gpio->port) |= _BV(gpio->portpin))
#define ONEWIRE_GPIO_READ(gpio)         (*(gpio->pin) & _BV(gpio->portpin))

/*** Timeouts ***/
#define ONEWIRE_TIMEOUT                 (125)

/*** Delays (see AVR318) ***/
#define ONEWIRE_DELAY_A                 (6)
#define ONEWIRE_DELAY_B                 (64)
#define ONEWIRE_DELAY_C                 (60)
#define ONEWIRE_DELAY_D                 (10)
#define ONEWIRE_DELAY_E                 (9)
#define ONEWIRE_DELAY_F                 (55)
#define ONEWIRE_DELAY_G                 (0)
#define ONEWIRE_DELAY_H                 (480)
#define ONEWIRE_DELAY_I                 (70)
#define ONEWIRE_DELAY_J                 (410)

/*** ROM-specific addresses ***/
#define ONEWIRE_ROM_SELECT              (0x55)
#define ONEWIRE_ROM_SKIP                (0xCC)
#define ONEWIRE_ROM_SEARCH              (0xF0)

/*** Search-specific ***/
#define ONEWIRE_SEARCH_FOUND            (1)
#define ONEWIRE_SEARCH_NOT_FOUND        (0)

/*** Parasite Power ***/
#define ONEWIRE_PARASITE_ON             (1)
#define ONEWIRE_PARASITE_OFF            (0)


/***** GLOBAL VARIABLES *******************************************************/


/***** ENUMERATION ************************************************************/


/***** STRUCTURES *************************************************************/
typedef struct {
    volatile uint8_t* ddr;
    volatile uint8_t* port;
    volatile uint8_t* pin;
    uint8_t portpin;
} onewire_t;


/***** FUNCTION PROTOTYPES ****************************************************/
/* General */
void onewire_get(onewire_t* gpio, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin);
uint8_t onewire_reset(onewire_t* gpio);
void onewire_depower(onewire_t* gpio);
/* Bit */
void onewire_write_bit(onewire_t* gpio, uint8_t value);
uint8_t onewire_read_bit(onewire_t* gpio);
/* Byte(s) */
void onewire_write_byte(onewire_t* gpio, uint8_t byte, uint8_t power);
void onewire_write(onewire_t* gpio, const uint8_t *bytes, uint16_t count, uint8_t power);
uint8_t onewire_read_byte(onewire_t* gpio);
void onewire_read(onewire_t* gpio, uint8_t *bytes, uint16_t count);
/* ROM */
void onewire_select(onewire_t* gpio, const uint8_t rom[8]);
void onewire_skip(onewire_t* gpio);


/* Search */
#if ONEWIRE_SEARCH
void onewire_search_reset(void);
void onewire_search_target(uint8_t family_code);
uint8_t onewire_search(onewire_t* gpio, uint8_t *addr);
#endif

/* CRC */
#if ONEWIRE_CRC
uint8_t onewire_crc8(const uint8_t *addr, uint8_t len);
#if ONEWIRE_CRC16
uint16_t onewire_crc16(const uint8_t* input, uint16_t len, uint16_t crc);
uint8_t onewire_crc16_check(const uint8_t* input, uint16_t len, const uint8_t* crc_i, uint16_t crc);
#endif
#endif


/***** INLINE FUNCTIONS *******************************************************/


#endif // _ONE_WIRE_H_
