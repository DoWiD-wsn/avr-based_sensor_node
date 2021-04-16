/*****
 * @brief   ASN(x) BME280 environmental sensor library
 *
 * Library to support the BME280 environmental sensor.
 *
 * @file    /_asnx_lib_/sensors/bme280.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/14 $
 * @see     https://github.com/bitbank2/bme280
 * @see     https://github.com/BoschSensortec/BME280_driver
 *****/


/***** INCLUDES *******************************************************/
#include "bme280.h"
/*** AVR ***/
#include <util/delay.h>
/*** ASNX LIB ***/
#include "i2c/i2c.h"


/***** LOCAL FUNCTION PROTOTYPES **************************************/
/* Calibration */
static BME280_RET_t _load_calibration(BME280_t* bme);
/* Raw readings */
static BME280_RET_t _wait_for_ready(BME280_t* bme);
static BME280_RET_t _get_temperature_raw(BME280_t* bme, uint32_t* value);
static BME280_RET_t _get_pressure_raw(BME280_t* bme, uint32_t* value);
static BME280_RET_t _get_humidity_raw(BME280_t* bme, uint16_t* value);


/***** FUNCTIONS ******************************************************/
/***
 * Initialization of a BME280 sensor instance.
 * 
 * @param[out]  bme         Pointer to the device structure to be filled
 * @param[in]   address     Device I2C address
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_init(BME280_t* bme, uint8_t address) {
    /* Initialize I2C master interface */
   i2c_init();
   /* Check if the device is available */
   if(i2c_is_available(address) != I2C_RET_OK) {
        /* Return ERROR */
        return BME280_RET_ERROR_NODEV;
   }
    
    /* Device is available ... store address in device structure */
    bme->address = address;
    
    /* Load calibration values */
    if(_load_calibration(bme) != BME280_RET_OK) {
        /* Loading calibration failed */
        return BME280_RET_ERROR;
    }
    /* Disable SPI interface */
    if(bme280_spi_disable(bme) != BME280_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Set the temperature sampling mode */
    if(bme280_set_t_sample(bme, BME280_SAMPLE_1) != BME280_RET_OK) {
        /* Loading calibration failed */
        return BME280_RET_ERROR;
    }
    /* Set the pressure sampling mode */
    if(bme280_set_p_sample(bme, BME280_SAMPLE_1) != BME280_RET_OK) {
        /* Loading calibration failed */
        return BME280_RET_ERROR;
    }
    /* Set the humidity sampling mode */
    if(bme280_set_h_sample(bme, BME280_SAMPLE_1) != BME280_RET_OK) {
        /* Loading calibration failed */
        return BME280_RET_ERROR;
    }
    /* Set the standby mode */
    if(bme280_set_standby(bme, BME280_STANDBY_250) != BME280_RET_OK) {
        /* Loading calibration failed */
        return BME280_RET_ERROR;
    }
    /* Set the filer mode */
    if(bme280_set_filter(bme, BME280_FILTER_OFF) != BME280_RET_OK) {
        /* Loading calibration failed */
        return BME280_RET_ERROR;
    }
    /* Set the mode of operation */
    if(bme280_set_mode(bme, BME280_MODE_NORMAL) != BME280_RET_OK) {
        /* Loading calibration failed */
        return BME280_RET_ERROR;
    }
    /* Set the initial t_fine value to 0.0 */
    bme->t_fine = 0.0;
    
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Read the BME280 chip ID.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[out]  value       BME280 chip ID
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_get_chipid(BME280_t* bme, uint8_t* value) {
    /* Read the value to the sensor */
    if(i2c_read_U8(bme->address, BME280_REG_CHIPID, value) != I2C_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Read the humidity-control register.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[out]  value       Humidity-control register value
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_get_ctrl_hum(BME280_t* bme, uint8_t* value) {
    /* Read the value to the sensor */
    if(i2c_read_U8(bme->address, BME280_REG_CTRL_HUM, value) != I2C_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Read the status register.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[out]  value       Status register value
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_get_status(BME280_t* bme, uint8_t* value) {
    /* Read the value to the sensor */
    if(i2c_read_U8(bme->address, BME280_REG_STATUS, value) != I2C_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Read the measurement control register.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[out]  value       Measurement control register value
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_get_ctrl_meas(BME280_t* bme, uint8_t* value) {
    /* Read the value to the sensor */
    if(i2c_read_U8(bme->address, BME280_REG_CTRL_MEAS, value) != I2C_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Read the configuration register.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[out]  value       Configuration register value
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_get_config(BME280_t* bme, uint8_t* value) {
    /* Read the value to the sensor */
    if(i2c_read_U8(bme->address, BME280_REG_CONFIG, value) != I2C_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Request a sensor reset.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_reset(BME280_t* bme) {
    /* Write the value to the sensor */
    if(i2c_write_8(bme->address, BME280_REG_RESET, BME280_RESET_VALUE) != I2C_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Set the mode of operation.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[in]   value       Mode of operation to be set
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_set_mode(BME280_t* bme, BME280_MODE_t value) {
    /* Set the value in the sensor configuration */
    bme->mode = value;
    /* Get the current register value */
    uint8_t reg;
    /* Get measurement control register */
    if(bme280_get_ctrl_meas(bme, &reg) != BME280_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Prepare new register value */
    reg = (reg & 0xFC) | (value<<0);
    /* Write the new value to the sensor */
    if(i2c_write_8(bme->address, BME280_REG_CTRL_MEAS, reg) != I2C_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Set the temperature sampling mode.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[in]   value       Temperature sampling mode to be set
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_set_t_sample(BME280_t* bme, BME280_SAMPLE_t value) {
    /* Set the value in the sensor configuration */
    bme->t_sample = value;
    /* Get the current register value */
    uint8_t reg;
    /* Get measurement control register */
    if(bme280_get_ctrl_meas(bme, &reg) != BME280_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Prepare new register value */
    reg = (reg & 0x1F) | (value<<5);
    /* Write the new value to the sensor */
    if(i2c_write_8(bme->address, BME280_REG_CTRL_MEAS, reg) != I2C_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Set the pressure sampling mode.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[in]   value       Pressure sampling mode to be set
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_set_p_sample(BME280_t* bme, BME280_SAMPLE_t value) {
    /* Set the value in the sensor configuration */
    bme->p_sample  = value;
    /* Get the current register value */
    uint8_t reg;
    /* Get measurement control register */
    if(bme280_get_ctrl_meas(bme, &reg) != BME280_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Prepare new register value */
    reg = (reg & 0xE3) | (value<<2);
    /* Write the new value to the sensor */
    if(i2c_write_8(bme->address, BME280_REG_CTRL_MEAS, reg) != I2C_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Set the humidity sampling mode.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[in]   value       Humidity sampling mode to be set
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_set_h_sample(BME280_t* bme, BME280_SAMPLE_t value) {
    /* Set the value in the sensor configuration */
    bme->h_sample  = value;
    /* Get the current register value */
    uint8_t reg;
    /* Get measurement control register */
    if(bme280_get_ctrl_hum(bme, &reg) != BME280_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Prepare new register value */
    reg = (reg & 0xF8) | value;
    /* Write the new value to the sensor */
    if(i2c_write_8(bme->address, BME280_REG_CTRL_HUM, reg) != I2C_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Set the standby mode.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[in]   value       Standby mode to be set
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_set_standby(BME280_t* bme, BME280_STANDBY_t value) {
    /* Set the value in the sensor configuration */
    bme->standby = value;
    /* Get the current register value */
    uint8_t reg;
    /* Get measurement control register */
    if(bme280_get_config(bme, &reg) != BME280_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Prepare new register value */
    reg = (reg & 0x1F) | (value<<5);
    /* Write the new value to the sensor */
    if(i2c_write_8(bme->address, BME280_REG_CONFIG, reg) != I2C_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Set the filter mode.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[in]   value       Filter mode to be set
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_set_filter(BME280_t* bme, BME280_FILTER_t value) {
    /* Set the value in the sensor configuration */
    bme->filter = value;
    /* Get the current register value */
    uint8_t reg;
    /* Get measurement control register */
    if(bme280_get_config(bme, &reg) != BME280_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Prepare new register value */
    reg = (reg & 0xE3) | (value<<2);
    /* Write the new value to the sensor */
    if(i2c_write_8(bme->address, BME280_REG_CONFIG, reg) != I2C_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Enable the SPI interface.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_spi_enable(BME280_t* bme) {
    /* Get the current register value */
    uint8_t reg;
    /* Get measurement control register */
    if(bme280_get_config(bme, &reg) != BME280_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Prepare new register value */
    reg = (reg & 0xFE) | BME280_SPI_ON;
    /* Write the new value to the sensor */
    if(i2c_write_8(bme->address, BME280_REG_CONFIG, reg) != I2C_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Disable the SPI interface.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_spi_disable(BME280_t* bme) {
    /* Get the current register value */
    uint8_t reg;
    /* Get measurement control register */
    if(bme280_get_config(bme, &reg) != BME280_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Prepare new register value */
    reg = (reg & 0xFE) | BME280_SPI_OFF;
    /* Write the new value to the sensor */
    if(i2c_write_8(bme->address, BME280_REG_CONFIG, reg) != I2C_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Get the compensated temperature reading in degree Celsius (°C).
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[out]  value       Compensated temperature reading in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_get_temperature(BME280_t* bme, float* value) {
    uint32_t raw;
    /* Get the raw temperature reading */
    if(_get_temperature_raw(bme, &raw) != BME280_RET_OK) {
        return BME280_RET_ERROR;
    }
    /* Calculate the compensated temperature value (see datasheet 8.2) */
    int32_t var1 = (((((int32_t)raw>>3) - ((int32_t)bme->calib.dig_T1<<1))) * ((int32_t)bme->calib.dig_T2)) >> 11;
    int32_t var2 = ((((((int32_t)raw>>4) - ((int32_t)bme->calib.dig_T1)) * (((int32_t)raw>>4) - ((int32_t)bme->calib.dig_T1))) >> 12) * ((int32_t)bme->calib.dig_T3)) >> 14;
    /* Calculate the fine resolution temperature value */
    bme->t_fine = var1+var2;
    /* Calculate and copy resulting value */
    *value = ((bme->t_fine * 5 + 128) >> 8) / 100.0;
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Get the compensated pressure reading in degree Celsius (°C).
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[out]  value       Compensated pressure reading in hectopascal (hPa)
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_get_pressure(BME280_t* bme, float* value) {
    uint32_t raw;
    /* Get the raw temperature reading */
    if(_get_pressure_raw(bme, &raw) != BME280_RET_OK) {
        return BME280_RET_ERROR;
    }
    /* Perform temperature measurement to update the t_fine value */
    float reading;
    if(bme280_get_temperature(bme, &reading) != BME280_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Calculate the compensated pressure value (see datasheet 8.2) */
    int32_t var1 = (((int32_t)bme->t_fine)>>1) - (int32_t)64000;
    int32_t var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)bme->calib.dig_P6);
    var2 = var2 + ((var1*((int32_t)bme->calib.dig_P5))<<1);
    var2 = (var2>>2) + (((int32_t)bme->calib.dig_P4)<<16);
    var1 = (((bme->calib.dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)bme->calib.dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768+var1)) * ((int32_t)bme->calib.dig_P1)) >> 15);
    /* Avoid division by zero */
    if(var1 == 0) {
        return 0; 
    }
    uint32_t p = (((uint32_t)(((int32_t)1048576)-(int32_t)raw)-(var2>>12)))*3125;
    if(p < 0x80000000) {
        p = (p << 1) / ((uint32_t)var1);
    } else {
        p = (p / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)bme->calib.dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
    var2 = (((int32_t)(p>>2)) * ((int32_t)bme->calib.dig_P8))>>13;
    /* Copy the resulting pressure in hPa */
    *value = ((int32_t)p + ((var1 + var2 + bme->calib.dig_P7) >> 4)) / 100.0;
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Get the compensated humidity reading in degree Celsius (°C).
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[out]  value       Compensated humidity reading in percent relative humidity (%RH)
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_get_humidity(BME280_t* bme, float* value) {
    uint16_t raw;
    /* Get the raw humidity reading */
    if(_get_humidity_raw(bme, &raw) != BME280_RET_OK) {
        return BME280_RET_ERROR;
    }
    /* Perform temperature measurement to update the t_fine value */
    float reading;
    if(bme280_get_temperature(bme, &reading) != BME280_RET_OK) {
        /* Writing failed */
        return BME280_RET_ERROR;
    }
    /* Calculate the compensated humidity value (see datasheet) */
    float humidity = (float)(bme->t_fine) - 76800.0;
    humidity = ((float)raw - ((float)(bme->calib.dig_H4) * 64.0 + (float)(bme->calib.dig_H5) / 16384.0 * humidity)) * ((float)(bme->calib.dig_H2) / 65536.0 * (1.0 + (float)(bme->calib.dig_H6) / 67108864.0 * humidity * (1.0 + (float)(bme->calib.dig_H3) / 67108864.0 * humidity)));
    humidity = humidity * (1.0 - (float)(bme->calib.dig_H1) * humidity / 524288.0);
    /* Check for value limits */
    if(humidity > 100.0) {
        humidity = 100.0;
    } else if(humidity < 0.0) {
        humidity = 0.0;
    }
    /* Copy the resulting value */
    *value = humidity;
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Get the calculated dew-point in degree Celsius (°C) (only accurate at > 50% RH).
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[out]  value       Calculated dew-point in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 ***/
BME280_RET_t bme280_get_dewpoint(BME280_t* bme, float* value) {
    float temperature;
    float humidity;
    /* Read temperature value */
    if(bme280_get_temperature(bme, &temperature) != BME280_RET_OK) {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* Read humidity value */
    if(bme280_get_humidity(bme, &humidity) != BME280_RET_OK) {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* Copy the dew-point */
    *value = temperature - ((100.0 - humidity) / 5.0);
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Load the calibration values stored on the sensor.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @return      OK in case of success; ERROR otherwise
 ***/
static BME280_RET_t _load_calibration(BME280_t* bme) {
    uint8_t tmp8U=0;
    int8_t tmp8S=0;
    uint16_t tmp16U=0;
    int16_t tmp16S=0;
    
    /*** temperature ***/
    /* BME280_REG_DIG_T1 */
    if(i2c_read_U16LE(bme->address, BME280_REG_DIG_T1, &tmp16U) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_T1 = tmp16U;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_T2 */
    if(i2c_read_S16LE(bme->address, BME280_REG_DIG_T2, &tmp16S) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_T2 = tmp16S;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_T3 */
    if(i2c_read_S16LE(bme->address, BME280_REG_DIG_T3, &tmp16S) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_T3 = tmp16S;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    
    /*** pressure ***/
    /* BME280_REG_DIG_P1 */
    if(i2c_read_U16LE(bme->address, BME280_REG_DIG_P1, &tmp16U) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_P1 = tmp16U;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_P2 */
    if(i2c_read_S16LE(bme->address, BME280_REG_DIG_P2, &tmp16S) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_P2 = tmp16S;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_P3 */
    if(i2c_read_S16LE(bme->address, BME280_REG_DIG_P3, &tmp16S) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_P3 = tmp16S;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_P4 */
    if(i2c_read_S16LE(bme->address, BME280_REG_DIG_P4, &tmp16S) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_P4 = tmp16S;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_P5 */
    if(i2c_read_S16LE(bme->address, BME280_REG_DIG_P5, &tmp16S) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_P5 = tmp16S;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_P6 */
    if(i2c_read_S16LE(bme->address, BME280_REG_DIG_P6, &tmp16S) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_P6 = tmp16S;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_P7 */
    if(i2c_read_S16LE(bme->address, BME280_REG_DIG_P7, &tmp16S) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_P7 = tmp16S;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_P8 */
    if(i2c_read_S16LE(bme->address, BME280_REG_DIG_P8, &tmp16S) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_P8 = tmp16S;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_P9 */
    if(i2c_read_S16LE(bme->address, BME280_REG_DIG_P9, &tmp16S) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_P9 = tmp16S;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    
    /*** humidity ***/
    /* BME280_REG_DIG_H1 */
    if(i2c_read_U8(bme->address, BME280_REG_DIG_H1, &tmp8U) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_H1 = tmp8U;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_H2 */
    if(i2c_read_S16LE(bme->address, BME280_REG_DIG_H2, &tmp16S) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_H2 = tmp16S;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_H3 */
    if(i2c_read_U8(bme->address, BME280_REG_DIG_H3, &tmp8U) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_H3 = tmp8U;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_H7 */
    if(i2c_read_S8(bme->address, BME280_REG_DIG_H7, &tmp8S) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_H6 = tmp8S;
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_H5 -> keep in tmp8U */
    if(i2c_read_U8(bme->address, BME280_REG_DIG_H5, &tmp8U) != I2C_RET_OK) {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_H4 */
    if(i2c_read_S8(bme->address, BME280_REG_DIG_H4, &tmp8S) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_H4 = ((int16_t)tmp8S << 4) | (tmp8U & 0x0F);
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* BME280_REG_DIG_H6 */
    if(i2c_read_S8(bme->address, BME280_REG_DIG_H6, &tmp8S) == I2C_RET_OK) {
        /* Store calibration value */
        bme->calib.dig_H5 = ((int16_t)tmp8S << 4) | (tmp8U & 0x0F);
    } else {
        /* Reading failed */
        return BME280_RET_ERROR;
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Check if the sensor readings are ready.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @return      OK in case of success; ERROR otherwise
 ***/
static BME280_RET_t _wait_for_ready(BME280_t* bme) {
    uint8_t value=0xFF;
    /* Wait for conversion to complete */
    uint16_t time_cnt=0;
    /* Read the status register */
    if(i2c_read_U8(bme->address, BME280_REG_STATUS, &value) != I2C_RET_OK) {
        /* Return with failure */
        return BME280_RET_ERROR;
    }
    while(value & BME280_STATUS_MEASURING) {
        /* Wait for 2ms */
        _delay_ms(2);
        /* Increment timeout counter */
        time_cnt+=1;
        /* Check if timeout has been reached */
        if(time_cnt > BME280_READ_TIMEOUT) {
            return BME280_RET_ERROR;
        }
        /* Read the status register again */
        if(i2c_read_U8(bme->address, BME280_REG_STATUS, &value) != I2C_RET_OK) {
            /* Return with failure */
            return BME280_RET_ERROR;
        }
    }
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Get the raw temperature reading from the sensor.
 * 
 * @param[in]   bme     Pointer to the device structure to be filled
 * @param[out]  value   Raw temperature reading
 * @return      OK in case of success; ERROR otherwise
 ***/
static BME280_RET_t _get_temperature_raw(BME280_t* bme, uint32_t* value) {
    /* Temporary result array */
    uint8_t  msb=0;
    uint8_t  lsb=0;
    uint8_t xlsb=0;
    /* Check if the sensor readings are ready */
    if(_wait_for_ready(bme) != BME280_RET_OK) {
        /* Return with failure */
        return BME280_RET_ERROR;
    }
    /* Read in temperature MSB */
    if(i2c_read_U8(bme->address, BME280_REG_T_MSB, &msb) != I2C_RET_OK) {
        /* Return with failure */
        return BME280_RET_ERROR;
    }
    /* Read in temperature LSB */
    if(i2c_read_U8(bme->address, BME280_REG_T_LSB, &lsb) != I2C_RET_OK) {
        /* Return with failure */
        return BME280_RET_ERROR;
    }
    /* Read in temperature XLSB */
    if(i2c_read_U8(bme->address, BME280_REG_T_XLSB, &xlsb) != I2C_RET_OK) {
        /* Return with failure */
        return BME280_RET_ERROR;
    }
    /* Calculate the raw temperature value */
    *value = (((uint32_t)msb << 16) | ((uint32_t)lsb << 8) | (uint32_t)xlsb) >> 4;
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Get the raw pressure reading from the sensor.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[out]  value       Raw pressure reading
 * @return      OK in case of success; ERROR otherwise
 ***/
static BME280_RET_t _get_pressure_raw(BME280_t* bme, uint32_t* value) {
    /* Temporary result array */
    uint8_t  msb=0;
    uint8_t  lsb=0;
    uint8_t xlsb=0;
    /* Check if the sensor readings are ready */
    if(_wait_for_ready(bme) != BME280_RET_OK) {
        /* Return with failure */
        return BME280_RET_ERROR;
    }
    /* Read in pressure MSB */
    if(i2c_read_U8(bme->address, BME280_REG_P_MSB, &msb) != I2C_RET_OK) {
        /* Return with failure */
        return BME280_RET_ERROR;
    }
    /* Read in pressure LSB */
    if(i2c_read_U8(bme->address, BME280_REG_P_LSB, &lsb) != I2C_RET_OK) {
        /* Return with failure */
        return BME280_RET_ERROR;
    }
    /* Read in pressure XLSB */
    if(i2c_read_U8(bme->address, BME280_REG_P_XLSB, &xlsb) != I2C_RET_OK) {
        /* Return with failure */
        return BME280_RET_ERROR;
    }
    /* Calculate the raw pressure value */
    *value = (((uint32_t)msb << 16) | ((uint32_t)lsb << 8) | (uint32_t)xlsb) >> 4;
    /* Return with success */
    return BME280_RET_OK;
}


/***
 * Get the raw humidity reading from the sensor.
 * 
 * @param[in]   bme         Pointer to the device structure to be filled
 * @param[out]  value       Raw humidity reading
 * @return      OK in case of success; ERROR otherwise
 ***/
static BME280_RET_t _get_humidity_raw(BME280_t* bme, uint16_t* value) {
    /* Temporary result array */
    uint8_t  msb=0;
    uint8_t  lsb=0;
    /* Check if the sensor readings are ready */
    if(_wait_for_ready(bme) != BME280_RET_OK) {
        /* Return with failure */
        return BME280_RET_ERROR;
    }
    /* Read in humidity MSB */
    if(i2c_read_U8(bme->address, BME280_REG_H_MSB, &msb) != I2C_RET_OK) {
        /* Return with failure */
        return BME280_RET_ERROR;
    }
    /* Read in humidity LSB */
    if(i2c_read_U8(bme->address, BME280_REG_H_LSB, &lsb) != I2C_RET_OK) {
        /* Return with failure */
        return BME280_RET_ERROR;
    }
    /* Calculate the raw pressure value */
    *value = ((uint32_t)msb << 8) | (uint32_t)lsb;
    /* Return with success */
    return BME280_RET_OK;
}
