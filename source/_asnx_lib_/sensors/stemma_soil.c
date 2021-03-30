/**
 *  Source file for STEMMA SOIL sensor.
 */

/***** INCLUDES ***************************************************************/
#include "stemma_soil.h"
#include <util/delay.h>


/***** FUNCTIONS **************************************************************/
/*
 * Initialization of the STEMMA SOIL sensor
 */
void stemma_init(void) {
    /* Initialize I2C master interface */
   i2c_init();
}


/*
 * Read the version code of the STEMMA SOIL sensor.
 * Bits [31:16] will be a date code, [15:0] will be the product id.
 */
stemma_ret_t stemma_get_version(uint32_t* version) {
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


/*
 * Read the temperature of the STEMMA SOIL sensor.
 * Temperature in degrees Celsius as a floating point value.
 */
stemma_ret_t stemma_get_temperature(float* temperature) {
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


/*
 * Read the capacity of the STEMMA SOIL sensor.
 * This is an integer between 0 and 1023.
 */
stemma_ret_t stemma_get_capacity(uint16_t* capacity) {
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
