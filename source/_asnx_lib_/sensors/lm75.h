/*****
 * @brief   ASN(x) LM75 temperature sensor library
 *
 * Library to support the LM75 temperature sensor.
 *
 * @file    /_asnx_lib_/sensors/lm75.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/14 $
 *
 * @todo    Add more functionality (i.e., configuration)
 *****/

#ifndef _ASNX_LM75_H_
#define _ASNX_LM75_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdio.h>
#include <stdint.h>


/***** DEFINES ********************************************************/
/* I2C address */
#define LM75_I2C_ADDRESS                0x48
#define LM75_I2C_ADDRESS_A0             (1<<0)
#define LM75_I2C_ADDRESS_A1             (1<<1)
#define LM75_I2C_ADDRESS_A2             (1<<2)
/* Shutdown configuration */
#define LM75_CONF_SHUTDOWN              0
/* Comparator/interrupt configuration */
#define LM75_CONF_COMP                  1
/* OS polarity configuration */
#define LM75_CONF_POL                   2
/* Fault queue configuration */
#define LM75_CONF_QUEUE                 3
#define LM75_CONF_QUEUE_MASK            0x18
#define LM75_CONF_QUEUE_1               0x00
#define LM75_CONF_QUEUE_2               0x01
#define LM75_CONF_QUEUE_4               0x02
#define LM75_CONF_QUEUE_6               0x03


/***** ENUMERATION ****************************************************/
/* Enumeration for the LM75 function return values */
typedef enum {
    LM75_RET_ERROR_NODEV    = -2,
    LM75_RET_ERROR          = -1,
    LM75_RET_OK             = 0
} LM75_RET_t;

/* Enumeration for the I2C register addresses */
typedef enum {
    LM75_REG_TEMP           = 0x00,
    LM75_REG_CONF           = 0x01,
    LM75_REG_HYST           = 0x02,
    LM75_REG_OS             = 0x03
} LM75_REG_t;


/***** STRUCTURES *****************************************************/
/***
 * A structure to store the LM75 module properties.
 ***/
typedef struct {
    uint8_t address;    /**< Sensors I2C address */
    uint8_t config;     /**< Sensor configuration (TODO: to be added) */
} LM75_t;


/***** FUNCTION PROTOTYPES ********************************************/
LM75_RET_t lm75_init(LM75_t* dev, uint8_t address);
/* Set */
LM75_RET_t lm75_set_config(LM75_t* dev, uint8_t value);
LM75_RET_t lm75_set_hyst(LM75_t* dev, float temp);
LM75_RET_t lm75_set_os(LM75_t* dev, float temp);
/* Get */
LM75_RET_t lm75_get_config(LM75_t* dev, uint8_t *value);
LM75_RET_t lm75_get_temperature(LM75_t* dev, float *temp);
LM75_RET_t lm75_get_hyst(LM75_t* dev, float *temp);
LM75_RET_t lm75_get_os(LM75_t* dev, float *temp);


#endif // _ASNX_LM75_H_
