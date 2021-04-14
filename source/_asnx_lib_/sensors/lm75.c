/*****
 * @brief   ASN(x) LM75 temperature sensor library
 *
 * Library to support the LM75 temperature sensor.
 *
 * @file    /_asnx_lib_/sensors/lm75.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/14 $
 *****/


/***** INCLUDES *******************************************************/
#include "lm75.h"
/*** ASNX LIB ***/
#include "i2c/i2c.h"


/***** LOCAL FUNCTION PROTOTYPE ***************************************/
float _reg2float(uint16_t value);
uint16_t _float2reg(float value);
LM75_RET_t _set_temp_value(float temp, uint8_t reg);
LM75_RET_t _get_temp_value(float *temp, uint8_t reg);


/***** FUNCTIONS ******************************************************/
/***
 * Initialization of the LM75 sensor.
 ***/
void lm75_init(void) {
    /* Initialize I2C master interface */
   i2c_init();
}


/***
 * Set the configuration register.
 * 
 * @param[in]   value       Configuration register value
 * @return      OK in case of success; ERROR otherwise
 ***/
LM75_RET_t lm75_set_config(uint8_t value) {
    /* Write the given value to the register */
    if(i2c_write_8(LM75_I2C_ADDRESS, LM75_REG_CONF, value) == I2C_RET_OK) {
        /* Writing was successful */
        return LM75_RET_OK;
    } else {
        /* Writing failed */
        return LM75_RET_ERROR;
    }
}


/***
 * Set the a temperature value to a given register.
 * 
 * @param[in]   temp        Temperature value in degree Celsius (°C)
 * @param[in]   reg         Configuration register address
 * @return      OK in case of success; ERROR otherwise
 ***/
LM75_RET_t _set_temp_value(float temp, uint8_t reg) {
    /* Convert float to word */
    uint16_t word = temp;           // TODO: needs to be fixed
    /* Swap bytes */
    word = ((word&0x00FF)<<8) | ((word&0xFF00)>>8);
    /* Write the given value to the register */
    if(i2c_write_16(LM75_I2C_ADDRESS, reg, word) == I2C_RET_OK) {
        /* Writing was successful */
        return LM75_RET_OK;
    } else {
        /* Writing failed */
        return LM75_RET_ERROR;
    }
}


/***
 * Set the hysteresis temperature.
 * 
 * @param[in]   temp        Hysteresis temperature value in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 ***/
LM75_RET_t lm75_set_hyst(float temp) {
    /* Write the temperature value */
    return _set_temp_value(temp,LM75_REG_HYST);
}


/***
 * Set the overtemperature shutdown temperature.
 * 
 * @param[in]   temp        Overtemperature shutdown temperature value in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 ***/
LM75_RET_t lm75_set_os(float temp) {
    /* Write the temperature value */
    return _set_temp_value(temp,LM75_REG_OS);
}


/***
 * Read the configuration register.
 * 
 * @param[out]  value       Data value memory location
 * @return      OK in case of success; ERROR otherwise
 ***/
LM75_RET_t lm75_get_config(uint8_t *value) {
    /* Read the config register */
    if(i2c_read_U8(LM75_I2C_ADDRESS, LM75_REG_CONF, value) == I2C_RET_OK) {
        /* Reading was successful */
        return LM75_RET_OK;
    } else {
        /* Reading failed */
        return LM75_RET_ERROR;
    }
}


/***
 * Get a temperature value from a given register.
 * 
 * @param[out]  temp        Read temperature value in degree Celsius (°C)
 * @param[in]   reg         Configuration register address
 * @return      OK in case of success; ERROR otherwise
 ***/
LM75_RET_t _get_temp_value(float *temp, uint8_t reg) {
    int16_t raw;
    /* Read the temperature register */
    if(i2c_read_S16BE(LM75_I2C_ADDRESS, reg, &raw) != I2C_RET_OK) {
        /* Reading failed */
        return LM75_RET_ERROR;
    }
    /* Convert temperature */
    *temp = ((float)raw / 32.0) / 8.0;
    /* Reading was successful */
    return LM75_RET_OK;
}


/***
 * Read the current sensor temperature.
 * 
 * @param[out]  temp        Current sensor temperature in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 ***/
LM75_RET_t lm75_get_temperature(float *temp) {
    /* Get the temperature value */
    return _get_temp_value(temp, LM75_REG_TEMP);
}


/***
 * Read the hysteresis temperature.
 * 
 * @param[out]  temp        Hysteresis temperature in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 ***/
LM75_RET_t lm75_get_hyst(float *temp) {
    /* Get the temperature value */
    return _get_temp_value(temp, LM75_REG_HYST);
}


/***
 * Read the overtemperature shutdown temperature.
 * 
 * @param[out]  temp        Overtemperature shutdown temperature in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 ***/
LM75_RET_t lm75_get_os(float *temp) {
    /* Get the temperature value */
    return _get_temp_value(temp, LM75_REG_OS);
}
