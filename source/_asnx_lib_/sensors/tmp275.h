/*****
 * @brief   ASN(x) TMP275 temperature sensor library
 *
 * Library to support the TMP275 temperature sensor.
 *
 * @file    /_asnx_lib_/sensors/tmp275.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/15 $
 *
 * @todo    Add more functionality (i.e., configuration)
 *****/

#ifndef _ASNX_TMP275_H_
#define _ASNX_TMP275_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdio.h>
#include <stdint.h>


/***** DEFINES ********************************************************/
/* Default I2C address */
#define TMP275_I2C_ADDRESS              0x48
/* Shutdown mode (SD) */
#define TMP275_CONF_SD_OFFSET           0
#define TMP275_CONF_SD_MASK             0x01
/* Thermostat mode (TM) */
#define TMP275_CONF_TM_OFFSET           1
#define TMP275_CONF_TM_MASK             0x02
/* Polarity (POL) */
#define TMP275_CONF_POL_OFFSET          2
#define TMP275_CONF_POL_MASK            0x04
/* Fault Queue (FX) */
#define TMP275_CONF_FX_OFFSET           3
#define TMP275_CONF_FX_MASK             0x18
/* Converter Resolution (RX) */
#define TMP275_CONF_RX_OFFSET           5
#define TMP275_CONF_RX_MASK             0x60
/* One-Shot (OS) */
#define TMP275_CONF_OS_OFFSET           7
#define TMP275_CONF_OS_MASK             0x80


/***** ENUMERATION ****************************************************/
/* Enumeration for the LM75 function return values */
typedef enum {
    TMP275_RET_ERROR_NODEV  = -2,
    TMP275_RET_ERROR        = -1,
    TMP275_RET_OK           = 0
} TMP275_RET_t;

/* Enumeration for the I2C register addresses */
typedef enum {
    TMP275_REG_TEMP         = 0x00,     /**< Temperature register */
    TMP275_REG_CONF         = 0x01,     /**< Configuration register */
    TMP275_REG_TLOW         = 0x02,     /**< Low limit register */
    TMP275_REG_THIGH        = 0x03      /**< High limit register */
} TMP275_REG_t;

/* Enumeration for the fault queue modes */
typedef enum {
    TMP275_CONF_FX_1        = 0x00,     /**< 1 consecutive fault */
    TMP275_CONF_FX_2        = 0x01,     /**< 2 consecutive faults */
    TMP275_CONF_FX_4        = 0x02,     /**< 4 consecutive faults */
    TMP275_CONF_FX_6        = 0x03      /**< 6 consecutive faults */
} TMP275_CONF_FX_t;

/* Enumeration for the resolution modes */
typedef enum {
    TMP275_CONF_RX_9        = 0x00,     /**< 9-bit; 27.5 ms conversion time */
    TMP275_CONF_RX_10       = 0x01,     /**< 10-bit; 55 ms conversion time */
    TMP275_CONF_RX_11       = 0x02,     /**< 11-bit; 110 ms conversion time */
    TMP275_CONF_RX_12       = 0x03      /**< 12-bit; 220 ms conversion time */
} TMP275_CONF_RX_t;


/***** STRUCTURES *****************************************************/
/***
 * A structure to store the TMP275 module properties.
 ***/
typedef struct {
    uint8_t address;    /**< Sensors I2C address */
    uint8_t config;     /**< Sensor configuration (TODO: to be added) */
} TMP275_t;


/***** FUNCTION PROTOTYPES ********************************************/
TMP275_RET_t tmp275_init(TMP275_t* dev, uint8_t address);
/* Set */
TMP275_RET_t tmp275_set_config(TMP275_t* dev, uint8_t value);
TMP275_RET_t tmp275_set_t_low(TMP275_t* dev, float temp);
TMP275_RET_t tmp275_set_t_high(TMP275_t* dev, float temp);
/* Get */
TMP275_RET_t tmp275_get_config(TMP275_t* dev, uint8_t *value);
TMP275_RET_t tmp275_get_temperature(TMP275_t* dev, float *temp);
TMP275_RET_t tmp275_get_t_low(TMP275_t* dev, float *temp);
TMP275_RET_t tmp275_get_t_high(TMP275_t* dev, float *temp);


#endif // _ASNX_TMP275_H_
