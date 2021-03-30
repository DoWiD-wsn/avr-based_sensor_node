/**
 *  Header file for STEMMA SOIL sensor.
 * 
 *  https://learn.adafruit.com/adafruit-stemma-soil-sensor-i2c-capacitive-moisture-sensor/
 *  https://github.com/adafruit/Adafruit_Seesaw
 */

#ifndef _SENS_STEMMA_H_
#define _SENS_STEMMA_H_

/***** INCLUDES ***************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "i2c/i2c.h"


/***** MACROS *****************************************************************/
/* I2C specific */
#define STEMMA_I2C_ADDRESS              (0x36)
/* I2C register addresses */
#define STEMMA_STATUS_BASE              (0x00)
#define STEMMA_STATUS_VERSION           (0x02)
#define STEMMA_STATUS_TEMP              (0x04)
#define STEMMA_TOUCH_BASE               (0x0F)
#define STEMMA_TOUCH_CHANNEL_BASE       (0x10)
#define STEMMA_TOUCH_CH0                (STEMMA_TOUCH_CHANNEL_BASE)
#define STEMMA_TOUCH_CH1                (STEMMA_TOUCH_CHANNEL_BASE+1)
#define STEMMA_TOUCH_CH2                (STEMMA_TOUCH_CHANNEL_BASE+2)
#define STEMMA_TOUCH_CH3                (STEMMA_TOUCH_CHANNEL_BASE+3)
/* Soil measurement status */
#define STEMMA_TOUCH_WORKING            (65535)


/***** ENUMERATION ************************************************************/
/* Function return values */
typedef enum {
                STEMMA_RET_ERROR = -1,
                STEMMA_RET_OK = 0
             } stemma_ret_t;


/***** FUNCTION PROTOTYPES ****************************************************/
void stemma_init(void);
stemma_ret_t stemma_get_version(uint32_t* version);
stemma_ret_t stemma_get_temperature(float* temperature);
stemma_ret_t stemma_get_capacity(uint16_t* capacity);


#endif // _SENS_STEMMA_H_
