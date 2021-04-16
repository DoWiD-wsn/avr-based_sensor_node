/*****
 * @brief   ASN(x) STEMMA SOIL sensor library
 *
 * Library to support the STEMMA SOIL sensor.
 *
 * @file    /_asnx_lib_/sensors/stemma_soil.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/14 $
 * @see     https://learn.adafruit.com/adafruit-stemma-soil-sensor-i2c-capacitive-moisture-sensor/
 * @see     https://github.com/adafruit/Adafruit_Seesaw
 *****/

#ifndef _ASNX_STEMMA_H_
#define _ASNX_STEMMA_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdio.h>
#include <stdint.h>
/*** ASNX LIB ***/
#include "i2c/i2c.h"


/***** DEFINES ********************************************************/
/* I2C address */
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
/* Soil measurement status */
#define STEMMA_TOUCH_WORKING            65535


/***** ENUMERATION ****************************************************/
/* Enumeration for the STEMMA function return values */
typedef enum {
    STEMMA_RET_ERROR_NODEV  = -2,
    STEMMA_RET_ERROR        = -1,
    STEMMA_RET_OK           = 0
} STEMMA_RET_t;


/***** STRUCTURES *****************************************************/
/***
 * A structure to store the STEMMA module properties.
 ***/
typedef struct {
    uint8_t address;    /**< Sensors I2C address */
    uint8_t config;     /**< Sensor configuration (TODO: to be added) */
} STEMMA_t;


/***** FUNCTION PROTOTYPES ****************************************************/
STEMMA_RET_t stemma_init(STEMMA_t* dev, uint8_t address);
STEMMA_RET_t stemma_get_version(STEMMA_t* dev, uint32_t* version);
STEMMA_RET_t stemma_get_temperature(STEMMA_t* dev, float* temperature);
STEMMA_RET_t stemma_get_capacity(STEMMA_t* dev, uint16_t* capacity);


#endif // _ASNX_STEMMA_H_
