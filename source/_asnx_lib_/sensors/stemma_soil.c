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


/***** INCLUDES *******************************************************/
#include "stemma_soil.h"
/*** AVR ***/
#include <util/delay.h>


/***** FUNCTIONS ******************************************************/
/***
 * Initialization of the STEMMA SOIL sensor.
 ***/
void stemma_init(void) {
    /* Initialize I2C master interface */
   i2c_init();
}


/***
 * Read the version code of the STEMMA SOIL sensor.
 * Bits [31:16] will be a date code, [15:0] will be the product id.
 * 
 * @param[out]  version         Version code read from the sensor
 * @return      OK in case of success; ERROR otherwise
 ***/
STEMMA_RET_t stemma_get_version(uint32_t* version) {
    /* Variable for the read result */
    uint8_t res[4] = {0};
    /* Read in 4 bytes */
    if(i2c_read16_block(STEMMA_I2C_ADDRESS, (STEMMA_STATUS_BASE | (STEMMA_STATUS_VERSION<<8)), res, 4) != I2C_RET_OK) {
        /* Reading failed */
        return STEMMA_RET_ERROR;
    }
    /* Copy the result */
    *version = ((uint32_t)res[0] << 24) | ((uint32_t)res[1] << 16) | ((uint32_t)res[2] << 8) | (uint32_t)res[3];
    /* Return success */
    return STEMMA_RET_OK;
}


/***
 * Read the temperature of the sensor in degrees Celsius (°C).
 * 
 * @param[out]  temperature      Sensor temperature in degrees Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 ***/
STEMMA_RET_t stemma_get_temperature(float* temperature) {
    /* Variable for the read result */
    uint8_t res[4] = {0};
    /* Read in 4 bytes */
    if(i2c_read16_block(STEMMA_I2C_ADDRESS, (STEMMA_STATUS_BASE | (STEMMA_STATUS_TEMP<<8)), res, 4) != I2C_RET_OK) {
        /* Reading failed */
        return STEMMA_RET_ERROR;
    }
    /* Get intermediate result */
    uint32_t ret = ((uint32_t)res[0] << 24) | ((uint32_t)res[1] << 16) | ((uint32_t)res[2] << 8) | (uint32_t)res[3];
    /* Copy the result */
    *temperature = (1.0 / (1UL << 16)) * (float)ret;
    /* Return success */
    return STEMMA_RET_OK;
}


/***
 * Read the capacity of the sensor represented as integer between 0 and 1023.
 * 
 * @param[out]  capacity        Sensor capacity (between 0 and 1023)
 * @return      OK in case of success; ERROR otherwise
 ***/
STEMMA_RET_t stemma_get_capacity(uint16_t* capacity) {
    /* Variable for the read result */
    uint16_t ret = STEMMA_TOUCH_WORKING;
    /* Capacitance measurement can take some time */
    do {
        /* Read in 2 bytes */
        if(i2c_read16_U16BE(STEMMA_I2C_ADDRESS, (STEMMA_TOUCH_BASE | (STEMMA_TOUCH_CH0<<8)), &ret) != I2C_RET_OK) {
            /* Reading failed */
            return STEMMA_RET_ERROR;
        }
    }while(ret == STEMMA_TOUCH_WORKING);
    /* Copy the result */
    *capacity = ret;
    /* Return success */
    return STEMMA_RET_OK;
}
