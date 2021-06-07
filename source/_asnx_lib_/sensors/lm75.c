/*!
 * @brief   ASN(x) LM75 temperature sensor library -- source file
 *
 * Library to support the LM75 temperature sensor.
 *
 * @file    /_asnx_lib_/sensors/lm75.c
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 *
 * @todo    Add more device functionality (i.e., configuration)
 */


/***** INCLUDES *******************************************************/
#include "lm75.h"


/***** LOCAL FUNCTION PROTOTYPE ***************************************/
static float _reg2float(uint16_t value);
static uint16_t _float2reg(float value);
static LM75_RET_t _set_temp_value(LM75_t* dev, float temp, uint8_t reg);
static LM75_RET_t _get_temp_value(LM75_t* dev, float *temp, uint8_t reg);


/***** FUNCTIONS ******************************************************/
/*!
 * Initialization of the LM75 sensor.
 *
 * @param[out]  dev         Pointer to the device structure to be filled
 * @param[in]   address     Device I2C address
 * @return      OK in case of success; ERROR otherwise
 */
LM75_RET_t lm75_init(LM75_t* dev, uint8_t address) {
   /* Check if the device is available */
   if(i2c_is_available(address) == I2C_RET_OK) {
        /* Device is available ... store address in device structure */
        dev->address = address;
        /* Initially, set config to default value */
        dev->config = 0x00;
        /* Return OK */
        return LM75_RET_OK;
   } else {
        /* Return ERROR */
        return LM75_RET_ERROR_NODEV;
   }
}


/*!
 * Convert a signed 16-bit word to float.
 *
 * @param[in]   value       signed 16-bit value
 * @return      Corresponding float value
 */
static float _reg2float(uint16_t value) {
    uint8_t hi = (uint8_t)((value&0xFF00)>>8);
    uint8_t lo = (uint8_t)(value&0x00FF);
    int16_t temp = ((hi << 4) | (lo >> 4));
	return (float)(temp/16.0);
}


/*!
 * Convert a float to signed 16-bit word.
 *
 * @param[in]   value       float value
 * @return      Corresponding signed 16-bit value
 */
static uint16_t _float2reg(float value) {
    int16_t temp = (int16_t)(value * 16.0);
    uint8_t hi = (uint8_t)((temp&0x000F)<<4);
    uint8_t lo = (uint8_t)((temp&0x0FF0)>>4);
    return ((hi<<8) | lo);
}

/*!
 * Set the configuration register.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[in]   value       Configuration register value
 * @return      OK in case of success; ERROR otherwise
 */
LM75_RET_t lm75_set_config(LM75_t* dev, uint8_t value) {
    /* Write the given value to the register */
    if(i2c_write_8(dev->address, LM75_REG_CONF, value) == I2C_RET_OK) {
        /* Writing was successful */
        return LM75_RET_OK;
    } else {
        /* Writing failed */
        return LM75_RET_ERROR;
    }
}


/*!
 * Set the a temperature value to a given register.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[in]   temp        Temperature value in degree Celsius (°C)
 * @param[in]   reg         Configuration register address
 * @return      OK in case of success; ERROR otherwise
 */
LM75_RET_t _set_temp_value(LM75_t* dev, float temp, uint8_t reg) {
    /* Convert float to word */
    uint16_t word = _float2reg(temp);
    /* Swap bytes */
    word = ((word&0x00FF)<<8) | ((word&0xFF00)>>8);
    /* Write the given value to the register */
    if(i2c_write_16(dev->address, reg, word) == I2C_RET_OK) {
        /* Writing was successful */
        return LM75_RET_OK;
    } else {
        /* Writing failed */
        return LM75_RET_ERROR;
    }
}


/*!
 * Set the hysteresis temperature.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[in]   temp        Hysteresis temperature value in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 */
LM75_RET_t lm75_set_hyst(LM75_t* dev, float temp) {
    /* Write the temperature value */
    return _set_temp_value(dev, temp, LM75_REG_HYST);
}


/*!
 * Set the overtemperature shutdown temperature.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[in]   temp        Overtemperature shutdown temperature value in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 */
LM75_RET_t lm75_set_os(LM75_t* dev, float temp) {
    /* Write the temperature value */
    return _set_temp_value(dev, temp, LM75_REG_OS);
}


/*!
 * Read the configuration register.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[out]  value       Data value memory location
 * @return      OK in case of success; ERROR otherwise
 */
LM75_RET_t lm75_get_config(LM75_t* dev, uint8_t *value) {
    /* Read the config register */
    if(i2c_read_U8(dev->address, LM75_REG_CONF, value) == I2C_RET_OK) {
        /* Reading was successful */
        return LM75_RET_OK;
    } else {
        /* Reading failed */
        return LM75_RET_ERROR;
    }
}


/*!
 * Get a temperature value from a given register.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[out]  temp        Read temperature value in degree Celsius (°C)
 * @param[in]   reg         Configuration register address
 * @return      OK in case of success; ERROR otherwise
 */
LM75_RET_t _get_temp_value(LM75_t* dev, float *temp, uint8_t reg) {
    int16_t raw;
    /* Read the temperature register */
    if(i2c_read_S16BE(dev->address, reg, &raw) != I2C_RET_OK) {
        /* Reading failed */
        return LM75_RET_ERROR;
    }
    /* Convert temperature */
    *temp = _reg2float(raw);
    /* Reading was successful */
    return LM75_RET_OK;
}


/*!
 * Read the current sensor temperature.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[out]  temp        Current sensor temperature in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 */
LM75_RET_t lm75_get_temperature(LM75_t* dev, float *temp) {
    /* Get the temperature value */
    return _get_temp_value(dev, temp, LM75_REG_TEMP);
}


/*!
 * Read the hysteresis temperature.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[out]  temp        Hysteresis temperature in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 */
LM75_RET_t lm75_get_hyst(LM75_t* dev, float *temp) {
    /* Get the temperature value */
    return _get_temp_value(dev, temp, LM75_REG_HYST);
}


/*!
 * Read the overtemperature shutdown temperature.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[out]  temp        Overtemperature shutdown temperature in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 */
LM75_RET_t lm75_get_os(LM75_t* dev, float *temp) {
    /* Get the temperature value */
    return _get_temp_value(dev, temp, LM75_REG_OS);
}
