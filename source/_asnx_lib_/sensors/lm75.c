/**
 *  Source file for LM75 sensor.
 */

/***** INCLUDES ***************************************************************/
#include "lm75.h"


/***** LOCAL FUNCTION PROTOTYPE ***********************************************/
float lm75_reg2float(uint16_t value);
uint16_t lm75_float2reg(float value);
lm75_ret_t lm75_set_temp_value(float temp, uint8_t reg);
lm75_ret_t lm75_get_temp_value(float *temp, uint8_t reg);


/***** FUNCTIONS **************************************************************/
/*
 * Initialization of the LM75 sensor
 */
void lm75_init(void) {
    /* Initialize I2C master interface */
   i2c_init();
}


/*** SET ***/
/*
 * Set the configuration register
 */
lm75_ret_t lm75_set_config(uint8_t value) {
    /* Write the given value to the register */
    if(i2c_write_8(LM75_I2C_ADDRESS, LM75_REG_CONF, value) == I2C_RET_OK) {
        /* Writing was successful */
        return LM75_RET_OK;
    } else {
        /* Writing failed */
        return LM75_RET_ERROR;
    }
}


/*
 * Set the a temperature value to a given register
 */
lm75_ret_t lm75_set_temp_value(float temp, uint8_t reg) {
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

/*
 * Set the hysteresis temperature
 */
lm75_ret_t lm75_set_hyst(float temp) {
    /* Write the temperature value */
    return lm75_set_temp_value(temp,LM75_REG_HYST);
}


/*
 * Set the overtemperature shutdown temperature
 */
lm75_ret_t lm75_set_os(float temp) {
    /* Write the temperature value */
    return lm75_set_temp_value(temp,LM75_REG_OS);
}


/*** GET ***/
/*
 * Read the configuration register
 */
lm75_ret_t lm75_get_config(uint8_t *value) {
    /* Read the config register */
    if(i2c_read_U8(LM75_I2C_ADDRESS, LM75_REG_CONF, value) == I2C_RET_OK) {
        /* Reading was successful */
        return LM75_RET_OK;
    } else {
        /* Reading failed */
        return LM75_RET_ERROR;
    }
}


/*
 * Get a temperature value from a given register
 */
lm75_ret_t lm75_get_temp_value(float *temp, uint8_t reg) {
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


/*
 * Read the current temperature
 */
lm75_ret_t lm75_get_temperature(float *temp) {
    /* Get the temperature value */
    return lm75_get_temp_value(temp, LM75_REG_TEMP);
}


/*
 * Read the hysteresis temperature
 */
lm75_ret_t lm75_get_hyst(float *temp) {
    /* Get the temperature value */
    return lm75_get_temp_value(temp, LM75_REG_HYST);
}


/*
 * Read the overtemperature shutdown temperature
 */
lm75_ret_t lm75_get_os(float *temp) {
    /* Get the temperature value */
    return lm75_get_temp_value(temp, LM75_REG_OS);
}
