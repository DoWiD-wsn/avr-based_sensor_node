/*!
 * @brief   ASN(x) one-wire interface (OWI) library -- header file
 *
 * Library to support one-wire interface (OWI) modules.
 * Globally deactivates interrupts during time-critical sections.
 *
 * @file    /_asnx_lib_/owi/owi.h
 * @author  Dominik Widhalm
 * @version 1.2.1
 * @date    2022/02/11
 * @see     https://github.com/szszoke/atmega328p/blob/master/onewire/
 * @see     https://hacksterio.s3.amazonaws.com/uploads/attachments/229743/OneWire.zip
 */

#ifndef _ASNX_OWI_H_
#define _ASNX_OWI_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
/*** AVR ***/
#include <avr/interrupt.h>
#include <util/delay.h>
/*** ASNX LIB ***/
#include "hw/hw.h"


/***** DEFINES ********************************************************/
/*** Enable/disable functionality ***/
/*! Exclude CRC checks by setting to (0) */
#define OWI_CRC                     (1)
/*! Select table-lookup for 8-bit CRC (1)
 * or use the comparably slower algorithm (0) */
#define OWI_CRC8_TABLE              (1)
/*! Allow 16-bit CRC checks by setting to (1) */
#define OWI_CRC16                   (1)
#if OWI_CRC
#  if OWI_CRC8_TABLE
#     include <avr/pgmspace.h>
#  endif
#endif

/*! ROM size in bytes */
#define OWI_ROM_SIZE                (8)

/*! OWI timeout (s) */
#define OWI_TIMEOUT                 (125)

/*** Delays (see application note AVR318) ***/
#define OWI_DELAY_A                 (6)     /**< AVR318 delay A: 6ms */
#define OWI_DELAY_B                 (64)    /**< AVR318 delay B: 64ms */
#define OWI_DELAY_C                 (60)    /**< AVR318 delay C: 60ms */
#define OWI_DELAY_D                 (10)    /**< AVR318 delay D: 10ms */
#define OWI_DELAY_E                 (9)     /**< AVR318 delay E: 9ms */
#define OWI_DELAY_F                 (55)    /**< AVR318 delay F: 55ms */
#define OWI_DELAY_G                 (0)     /**< AVR318 delay G: 0ms */
#define OWI_DELAY_H                 (480)   /**< AVR318 delay H: 480ms */
#define OWI_DELAY_I                 (70)    /**< AVR318 delay I: 70ms */
#define OWI_DELAY_J                 (410)   /**< AVR318 delay J: 410ms */

/*** ROM-specific addresses ***/
#define OWI_ROM_SELECT              (0x55)  /**< OWI ROM select address */
#define OWI_ROM_SKIP                (0xCC)  /**< OWI ROM skip address */
#define OWI_ROM_SEARCH              (0xF0)  /**< OWI ROM search address */



/***** STRUCTURES *****************************************************/
/*!
 * A structure to store the search properties.
 */
typedef struct {
    uint8_t ROM_NO[OWI_ROM_SIZE];       /**< ROM data */
    uint8_t last_discrepancy;           /**< Flag for last discrepancy */
    uint8_t last_family_discrepancy;    /**< Flag for last family discrepancy */
    uint8_t last_device_flag;           /**< Flag for last device */
} OWI_DATA_t;


/***** ENUMERATION ****************************************************/
/*! Enumeration for the OWI search return values */
typedef enum {
    OWI_SEARCH_NOT_FOUND    = 0,
    OWI_SEARCH_FOUND        = 1
} OWI_SEARCH_t;

/*! Enumeration for the OWI parasitic power mode */
typedef enum {
    OWI_PARASITE_OFF        = 0,
    OWI_PARASITE_ON         = 1
} OWI_POWER_t;


/***** FUNCTION PROTOTYPES ********************************************/
/* General */
void owi_get(hw_io_t* gpio, OWI_DATA_t* data, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin);
uint8_t owi_reset(hw_io_t* gpio);
void owi_depower(hw_io_t* gpio);
/* Bit */
void owi_write_bit(hw_io_t* gpio, uint8_t value);
uint8_t owi_read_bit(hw_io_t* gpio);
/* Byte(s) */
void owi_write_byte(hw_io_t* gpio, uint8_t byte, OWI_POWER_t power);
void owi_write(hw_io_t* gpio, const uint8_t *bytes, uint16_t count, OWI_POWER_t power);
uint8_t owi_read_byte(hw_io_t* gpio);
void owi_read(hw_io_t* gpio, uint8_t *bytes, uint16_t count);
/* ROM */
void owi_select(hw_io_t* gpio, uint8_t *addr);
void owi_skip(hw_io_t* gpio);


/* Search */
void owi_search_reset(OWI_DATA_t* data);
void owi_search_target(OWI_DATA_t* data, uint8_t family_code);
OWI_SEARCH_t owi_search(hw_io_t* gpio, OWI_DATA_t* data, uint8_t *addr);

/* CRC */
#if OWI_CRC
uint8_t owi_crc8(const uint8_t *addr, uint8_t len);
#if OWI_CRC16
uint16_t owi_crc16(const uint8_t* input, uint16_t len, uint16_t crc);
uint8_t owi_crc16_check(const uint8_t* input, uint16_t len, const uint8_t* crc_i, uint16_t crc);
#endif
#endif


#endif // _ASNX_OWI_H_
