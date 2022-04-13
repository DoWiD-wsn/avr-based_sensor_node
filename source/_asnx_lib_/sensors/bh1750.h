/*!
 * @brief   ASN(x) BH1750 sensor library -- header file
 *
 * Library to support the BH1750 sensor (via I2C).
 *
 * @file    /_asnx_lib_/sensors/bh1750.h
 * @author  Dominik Widhalm
 * @version 1.0.0
 * @date    2022/04/13
 */

#ifndef _ASNX_BH1750_H_
#define _ASNX_BH1750_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
/*** AVR ***/
#include <util/delay.h>
/*** ASNX LIB ***/
#include "hw/hw.h"
#include "i2c/i2c.h"


/***** DEFINES ********************************************************/
/*** I2C General ***/
/*! I2C default address */
#define BH1750_I2C_ADDRESS              0x23

/*** DVI DEFAULT GPIO ***/
/*! DVI default DDR register */
#define BH1750_DVI_DDR                  DDRC
/*! DVI default PORT register */
#define BH1750_DVI_PORT                 PORTC
/*! DVI default PIN register */
#define BH1750_DVI_PIN                  PINC
/*! DVI default portpin index */
#define BH1750_DVI_GPIO                 PC5

/*** Sensor specific ***/
/*! Asynchronous reset delay (DVI) [us] (datasheet: more than 1us) */
#define BH1750_RESET_DELAY              5

/*** Measurement setting */
/*! Measurement resolution (L=0; H=1; H2=2; default: 0) */
#define BH1750_MEASURE_RESOLUTION       0
/*! Measurement mode (One-Time=0; Continuous=1; default: 0) */
#define BH1750_MEASURE_MODE             0

/*** I2C commands ***/
// General
/*! Power Down (0000_0000) */
#define BH1750_COM_POWER_DOWN           0x00
/*! Power On (0000_0001) */
#define BH1750_COM_POWER_ON             0x01
/*! Reset (0000_0111) */
#define BH1750_COM_RESET                0x07
// Continuous
/*! Continuously H-Resolution Mode (Typ. 120ms; 0.5 lx -> 0001_0000) */
#define BH1750_COM_CON_H                0x10
/*! Continuously H-Resolution Mode2 (Typ. 120ms; 1 lx -> 0001_0001) */
#define BH1750_COM_CON_H2               0x11
/*! Continuously L-Resolution Mode (Typ. 16ms; 4 lx -> 0001_0011) */
#define BH1750_COM_CON_L                0x13
// One-Time
/*! One-Time H-Resolution Mode (Typ. 120ms; 0.5 lx -> 0010_0000) */
#define BH1750_COM_OT_H                 0x20
/*! One-Time H-Resolution Mode2 (Typ. 120ms; 1 lx -> 0010_0001) */
#define BH1750_COM_OT_H2                0x21
/*! One-Time L-Resolution Mode (Typ. 16ms; 4 lx -> 0010_0011) */
#define BH1750_COM_OT_L                 0x23
// Config
/*! Change Measurement time (High bit -> 01000_MT[7,6,5]) */
#define BH1750_COM_CHANGE_HIGH_MASK     0x40
/*! Change Measurement time (Low bit -> 011_MT[4,3,2,1,0]) */
#define BH1750_COM_CHANGE_LOW_MASK      0x60

/*! Actual read command depends on measurement settings */
// One-time mode
#if BH1750_MEASURE_MODE==0
#  if BH1750_MEASURE_RESOLUTION==0
#    define BH1750_MEASURE          BH1750_COM_OT_L
#    define BH1750_MEASURE_DELAY_MS 24
#  elif BH1750_MEASURE_RESOLUTION==1
#    define BH1750_MEASURE          BH1750_COM_OT_H
#    define BH1750_MEASURE_DELAY_MS 180
#  else
#    define BH1750_MEASURE          BH1750_COM_OT_H2
#    define BH1750_MEASURE_DELAY_MS 180
#  endif
// Continuous mode
#else
#  if BH1750_MEASURE_RESOLUTION==0
#    define BH1750_MEASURE          BH1750_COM_CON_L
#    define BH1750_MEASURE_DELAY_MS 24
#  elif BH1750_MEASURE_RESOLUTION==1
#    define BH1750_MEASURE          BH1750_COM_CON_H
#    define BH1750_MEASURE_DELAY_MS 180
#  else
#    define BH1750_MEASURE          BH1750_COM_CON_H2
#    define BH1750_MEASURE_DELAY_MS 180
#  endif
#endif

/*** BH1750 function return values ***/
#define BH1750_RET_ERROR_NODEV      (-2)
#define BH1750_RET_ERROR            (-1)
#define BH1750_RET_OK               (0)


/***** STRUCTURES *****************************************************/
/*!
 * A structure to store the BH1750 module properties.
 */
typedef struct {
    uint8_t address;    /**< Sensors I2C address */
    hw_io_t dvi;        /**< DVI GPIO handle */
} BH1750_t;


/***** FUNCTION PROTOTYPES ********************************************/
int8_t bh1750_init_default(BH1750_t* dev);
int8_t bh1750_init(BH1750_t* dev, uint8_t address, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin);
/* General */
int8_t bh1750_sleep_enable(BH1750_t* dev);
int8_t bh1750_sleep_disable(BH1750_t* dev);
int8_t bh1750_reset(BH1750_t* dev);
void bh1750_reset_async(BH1750_t* dev);
/* Sensor readings */
int8_t bh1750_get_illuminance(BH1750_t* dev, uint16_t* illuminance);

#endif // _ASNX_BH1750_H_
