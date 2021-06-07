/*!
 * @brief   ASN(x) STEMMA SOIL sensor library -- header file
 *
 * Library to support the STEMMA SOIL sensor.
 *
 * @file    /_asnx_lib_/sensors/stemma_soil.h
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 *
 * @see     https://learn.adafruit.com/adafruit-stemma-soil-sensor-i2c-capacitive-moisture-sensor/
 * @see     https://github.com/adafruit/Adafruit_Seesaw
 */

#ifndef _ASNX_STEMMA_H_
#define _ASNX_STEMMA_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
/*** AVR ***/
#include <util/delay.h>
/*** ASNX LIB ***/
#include "i2c/i2c.h"


/***** DEFINES ********************************************************/
/*! Maximum capacitance value */
#define STEMMA_CAP_MAX                  1017.0
/*! Minimum capacitance value */
#define STEMMA_CAP_MIN                  300.0

/*! Number of measurements for floating average calculation */
#define STEMMA_AVG_CNT                  10

/*! I2C default address */
#define STEMMA_I2C_ADDRESS              0x36
/* I2C register addresses */
#define STEMMA_STATUS_BASE              0x00
#define STEMMA_STATUS_VERSION           0x02
#define STEMMA_STATUS_TEMP              0x04
#define STEMMA_TOUCH_BASE               0x0F
#define STEMMA_TOUCH_CHANNEL_BASE       0x10
#define STEMMA_TOUCH_CH0                (STEMMA_TOUCH_CHANNEL_BASE)
#define STEMMA_TOUCH_CH1                (STEMMA_TOUCH_CHANNEL_BASE+1)
#define STEMMA_TOUCH_CH2                (STEMMA_TOUCH_CHANNEL_BASE+2)
#define STEMMA_TOUCH_CH3                (STEMMA_TOUCH_CHANNEL_BASE+3)
/*! Soil measurement status */
#define STEMMA_TOUCH_WORKING            65535


/***** ENUMERATION ****************************************************/
/*! Enumeration for the STEMMA function return values */
typedef enum {
    STEMMA_RET_ERROR_NODEV  = -2,
    STEMMA_RET_ERROR        = -1,
    STEMMA_RET_OK           = 0
} STEMMA_RET_t;


/***** STRUCTURES *****************************************************/
/*!
 * A structure to store the STEMMA module properties.
 */
typedef struct {
    uint8_t address;                /**< Sensors I2C address */
} STEMMA_t;

/*!
 * A structure to store measurements for the floating average calculation.
 */
typedef struct {
    uint8_t empty;                  /**< Flag to indicate the first reading (0 ... first; otherwise 1) */
    float value[STEMMA_AVG_CNT];    /**< Sensor readings */
} STEMMA_AVG_t;


/***** FUNCTION PROTOTYPES ****************************************************/
STEMMA_RET_t stemma_init(STEMMA_t* dev, uint8_t address);
STEMMA_RET_t stemma_get_version(STEMMA_t* dev, uint32_t* version);
STEMMA_RET_t stemma_get_temperature(STEMMA_t* dev, float* temperature);
STEMMA_RET_t stemma_get_humidity(STEMMA_t* dev, float* humidity);
STEMMA_RET_t stemma_get_humidity_avg(STEMMA_t* dev, STEMMA_AVG_t* structure, float* humidity);


#endif // _ASNX_STEMMA_H_
