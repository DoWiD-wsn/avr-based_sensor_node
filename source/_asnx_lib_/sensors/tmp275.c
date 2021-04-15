/*****
 * @brief   ASN(x) TMP275 temperature sensor library
 *
 * Library to support the TMP275 temperature sensor.
 *
 * @file    /_asnx_lib_/sensors/tmp275.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/15 $
 *
 * @todo    Add more functionality (i.e., configuration)
 *****/


/***** INCLUDES *******************************************************/
#include "tmp275.h"
/*** ASNX LIB ***/
#include "i2c/i2c.h"


/***** LOCAL FUNCTION PROTOTYPE ***************************************/
float _reg2float(uint16_t value);
uint16_t _float2reg(float value);
TMP275_RET_t tmp275_set_temp_value(float temp, uint8_t reg);
TMP275_RET_t tmp275_get_temp_value(float *temp, uint8_t reg);


/***** FUNCTIONS ******************************************************/
/***
 * Initialization of the TMP275 sensor.
 ***/
void tmp275_init(void) {
    /* Initialize I2C master interface */
   i2c_init();
}


/***
 * Convert a signed 16-bit word to float.
 *
 * @param[in]   value       signed 16-bit value
 * @return      Corresponding float value
 ***/
float _reg2float(uint16_t value) {
    uint8_t hi = (uint8_t)((value&0xFF00)>>8);
    uint8_t lo = (uint8_t)(value&0x00FF);
    int16_t temp = ((hi << 4) | (lo >> 4));
	return (float)(temp/16.0);
    //return 0.0625 * ((int16_t)value >> 4);
}


/***
 * Convert a float to signed 16-bit word.
 *
 * @param[in]   value       float value
 * @return      Corresponding signed 16-bit value
 ***/
uint16_t _float2reg(float value) {
    int16_t temp = (int16_t)(value * 16.0);
    uint8_t hi = (uint8_t)((temp&0x000F)<<4);
    uint8_t lo = (uint8_t)((temp&0x0FF0)>>4);
    return ((hi<<8) | lo);
}


/***
 * Set the configuration register.
 * 
 * @param[in]   value       Configuration register value
 * @return      OK in case of success; ERROR otherwise
 ***/
TMP275_RET_t tmp275_set_config(uint8_t value) {
    /* Write the given value to the register */
    if(i2c_write_8(TMP275_I2C_ADDRESS, TMP275_REG_CONF, value) == I2C_RET_OK) {
        /* Writing was successful */
        return TMP275_RET_OK;
    } else {
        /* Writing failed */
        return TMP275_RET_ERROR;
    }
}


/***
 * Set the a temperature value to a given register.
 * 
 * @param[in]   temp        Temperature value in degree Celsius (°C)
 * @param[in]   reg         Configuration register address
 * @return      OK in case of success; ERROR otherwise
 ***/
TMP275_RET_t tmp275_set_temp_value(float temp, uint8_t reg) {
    /* Convert float to word */
    uint16_t word = _float2reg(temp);
    /* Swap bytes */
    word = ((word&0x00FF)<<8) | ((word&0xFF00)>>8);
    /* Write the given value to the register */
    if(i2c_write_16(TMP275_I2C_ADDRESS, reg, word) == I2C_RET_OK) {
        /* Writing was successful */
        return TMP275_RET_OK;
    } else {
        /* Writing failed */
        return TMP275_RET_ERROR;
    }
}


/***
 * Set the low limit temperature.
 * 
 * @param[in]   temp        Low limit temperature value in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 ***/
TMP275_RET_t tmp275_set_t_low(float temp) {
    /* Write the temperature value */
    return tmp275_set_temp_value(temp,TMP275_REG_TLOW);
}


/***
 * Set the high limit temperature.
 * 
 * @param[in]   temp        High limit temperature value in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 ***/
TMP275_RET_t tmp275_set_t_high(float temp) {
    /* Write the temperature value */
    return tmp275_set_temp_value(temp,TMP275_REG_THIGH);
}


/***
 * Read the configuration register.
 * 
 * @param[out]  value       Data value memory location
 * @return      OK in case of success; ERROR otherwise
 ***/
TMP275_RET_t tmp275_get_config(uint8_t *value) {
    /* Read the config register */
    if(i2c_read_U8(TMP275_I2C_ADDRESS, TMP275_REG_CONF, value) == I2C_RET_OK) {
        /* Reading was successful */
        return TMP275_RET_OK;
    } else {
        /* Reading failed */
        return TMP275_RET_ERROR;
    }
}


/***
 * Get a temperature value from a given register.
 * 
 * @param[out]  temp        Read temperature value in degree Celsius (°C)
 * @param[in]   reg         Configuration register address
 * @return      OK in case of success; ERROR otherwise
 ***/
TMP275_RET_t tmp275_get_temp_value(float *temp, uint8_t reg) {
    int16_t raw;
    /* Read the temperature register */
    if(i2c_read_S16BE(TMP275_I2C_ADDRESS, reg, &raw) != I2C_RET_OK) {
        /* Reading failed */
        return TMP275_RET_ERROR;
    }
    /* Convert raw reading to temperature */
    *temp = _reg2float(raw);
    /* Reading was successful */
    return TMP275_RET_OK;
}


/***
 * Read the current sensor temperature.
 * 
 * @param[out]  temp        Current sensor temperature in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 ***/
TMP275_RET_t tmp275_get_temperature(float *temp) {
    /* Get the temperature value */
    return tmp275_get_temp_value(temp, TMP275_REG_TEMP);
}


/***
 * Read the low limit temperature.
 * 
 * @param[out]  temp        Low limit temperature in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 ***/
TMP275_RET_t tmp275_get_t_low(float *temp) {
    /* Get the temperature value */
    return tmp275_get_temp_value(temp, TMP275_REG_TLOW);
}


/***
 * Read the high limit temperature.
 * 
 * @param[out]  temp        High limit temperature in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 ***/
TMP275_RET_t tmp275_get_t_high(float *temp) {
    /* Get the temperature value */
    return tmp275_get_temp_value(temp, TMP275_REG_THIGH);
}
