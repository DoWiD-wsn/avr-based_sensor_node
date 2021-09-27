/*!
 * @brief   ASN(x) SHTC3 sensor library -- header file
 *
 * Library to support the SHTC3 sensor (via I2C).
 *
 * @file    /_asnx_lib_/sensors/shtc3.h
 * @author  Dominik Widhalm
 * @version 1.0.0
 * @date    2021/09/27
 * 
 * @see     https://github.com/Sensirion/shtc3-stm-sample-project/
 * @note    Based on the SHTC3 sample code for STM32 by Sensirion
 * @note    Clock stretching needs to be supported by the device (AVR does not!)
 */

#ifndef _ASNX_SHTC3_H_
#define _ASNX_SHTC3_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
/*** AVR ***/
#include <util/delay.h>
/*** ASNX LIB ***/
#include "i2c/i2c.h"

/***** DEFINES ********************************************************/
/*** I2C General ***/
/*! I2C default address */
#define SHTC3_I2C_ADDRESS               0x70

/*** Sensor specific ***/
/*! Delay after requesting the sensor to wake-up [ms] */
#define SHTC3_WAKEUP_DELAY              100
/*! Maximum number of retries for polling */
#define SHTC3_POLLING_MAX               20
/*! CRC polynomial: P(x) = x^8 + x^5 + x^4 + 1 = 100110001 */
#define SHTC3_CRC_POLYNOMIAL            0x131

/*** Measurement setting */
/*! Measure RH before T (default: 0) */
#define SHTC3_TH_FIRST                  0
/*! Use clock stretching during measurement (default: 0; check for device support) */
#define SHTC3_CLOCK_STRETCHING          0

/*** I2C commands ***/
/*! Read ID register */
#define SHTC3_COM_READID                0xEFC8
/*! Soft reset */
#define SHTC3_COM_SOFT_RESET            0x805D
/*! Sleep */
#define SHTC3_COM_SLEEP                 0xB098
/*! Wakeup */
#define SHTC3_COM_WAKEUP                0x3517
/*! Measure T->RH with polling */
#define SHTC3_COM_MEAS_NORM             0x7866
/*! Measure T->RH with clock stretching */
#define SHTC3_COM_MEAS_NORM_CLKSTR      0x7CA2
/*! Measure RH->T with polling */
#define SHTC3_COM_MEAS_REV              0x58E0
/*! Measure RH->T with clock stretching */
#define SHTC3_COM_MEAS_REV_CLKSTR       0x5C24
/* Actual read command depends on settings */
#if SHTC3_TH_FIRST
#  if SHTC3_CLOCK_STRETCHING
#    define SHTC3_COM_MEAS      SHTC3_COM_MEAS_REV_CLKSTR
#  else
#    define SHTC3_COM_MEAS      SHTC3_COM_MEAS_REV
#  endif
#else
#  if SHTC3_CLOCK_STRETCHING
#    define SHTC3_COM_MEAS      SHTC3_COM_MEAS_NORM_CLKSTR
#  else
#    define SHTC3_COM_MEAS      SHTC3_COM_MEAS_NORM
#  endif
#endif


/***** ENUMERATION ****************************************************/
/*! Enumeration for the SHTC3 function return values */
typedef enum {
    SHTC3_RET_ERROR_CRC    = -3,
    SHTC3_RET_ERROR_NODEV  = -2,
    SHTC3_RET_ERROR        = -1,
    SHTC3_RET_OK           = 0
} SHTC3_RET_t;


/***** STRUCTURES *****************************************************/
/*!
 * A structure to store the SHTC3 module properties.
 */
typedef struct {
    uint8_t address;    /**< Sensors I2C address */
} SHTC3_t;


/***** FUNCTION PROTOTYPES ********************************************/
SHTC3_RET_t shtc3_init(SHTC3_t* dev, uint8_t address);
/* General */
SHTC3_RET_t shtc3_reset(SHTC3_t* dev);
SHTC3_RET_t shtc3_get_id(SHTC3_t* dev, uint16_t *id);
SHTC3_RET_t shtc3_sleep_enable(SHTC3_t* dev);
SHTC3_RET_t shtc3_sleep_disable(SHTC3_t* dev);
/* Sensor readings */
SHTC3_RET_t shtc3_get_temperature(SHTC3_t* dev, float* temperature);
SHTC3_RET_t shtc3_get_humidity(SHTC3_t* dev, float* humidity);
SHTC3_RET_t shtc3_get_temperature_humidity(SHTC3_t* dev, float* temperature, float* humidity);

#endif // _ASNX_SHTC3_H_
