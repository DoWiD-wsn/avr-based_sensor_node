/*!
 * @brief   ASN(x) SHT30 sensor library -- source file
 *
 * Library to support the SHT30 sensor (via I2C).
 *
 * @file    /_asnx_lib_/sensors/sht30.c
 * @author  Dominik Widhalm
 * @version 1.0.0
 * @date    2022/04/13
 */


/***** INCLUDES *******************************************************/
#include "sht30.h"


/***** LOCAL FUNCTION PROTOTYPE ***************************************/
int8_t _read16_crc(uint16_t* value);
static int8_t _check_crc(uint8_t* data, uint8_t len, uint8_t checksum);

static float _raw2temperature(uint16_t raw);
static float _raw2humidity(uint16_t raw);


/***** FUNCTIONS ******************************************************/


/***** INTERNAL ***********************/
/*!
 * Read 16-bit from the SHT30 sensor and perform CRC check.
 *
 * @param[out]  value       Pointer to the location of the 16-bit variable to be read
 * @return      OK in case of success; ERROR otherwise
 */
int8_t _read16_crc(uint16_t* value) {
    /* Temporary data bytes */
    uint8_t data[2];
    /* Checksum byte */
    uint8_t checksum;
    
    /* Read two data bytes and one checksum byte */
    if(i2c_get_ack(&data[0]) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    if(i2c_get_ack(&data[1]) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    if(i2c_get_ack(&checksum) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    
    /* Verify checksum */
    if(_check_crc(data, 2, checksum) != SHT30_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR_CRC;
    }
    /* Combine the two bytes to a 16-bit value */
    *value = (data[0] << 8) | data[1];
    /* Return OK */
    return SHT30_RET_OK;
}


/*!
 * Perform a CRC check.
 *
 * @param[in]   data        Input data to be checked
 * @param[in]   len         Number of bytes to be checked
 * @param[in]   checksum    Received CRC checksum
 * @return      OK in case of success; ERROR otherwise
 */
static int8_t _check_crc(uint8_t* data, uint8_t len, uint8_t checksum) {
    uint8_t bit;            /* bit mask */
    uint8_t crc = 0xFF;     /* Calculated checksum */
    uint8_t cnt;            /* byte counter */

    /* Calculate CRC with given polynomial */
    for(cnt=0; cnt<len; cnt++) {
        crc ^= (data[cnt]);
        for(bit=8; bit>0; --bit) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ SHT30_CRC_POLYNOMIAL;
            } else {
                crc = (crc << 1);
            }
        }
    }

    /* Verify checksum */
    if(crc != checksum) {
        return SHT30_RET_ERROR_CRC;
    } else {
        return SHT30_RET_OK;
    }
}


/*!
 * Convert a raw temperature to degrees Celsius.
 * T = -45 + 175 * raw / 2^16
 *
 * @param[in]   raw         Raw temperature reading
 * @return      Resulting temperature in degrees Celsius
 */
static float _raw2temperature(uint16_t raw) {
  /* Calculate resulting temperature [°C] */
  return 175.0 * ((float)raw / 65536.0) - 45.0;
}


/*!
 * Convert a raw humidity to percent relative humidity.
 * RH = raw / 2^16 * 100
 *
 * @param[in]   raw         Raw humidity reading
 * @return      Resulting humidity in percent relative humidity
 */
static float _raw2humidity(uint16_t raw) {
  /* Calculate resulting relative humidity [%RH] */
  return 100.0 * ((float)raw / 65536.0);
}


/***** GENERAL ************************/
/*!
 * Initialization of the SHT30 sensor.
 *
 * @param[out]  dev         Pointer to the device structure to be filled
 * @param[in]   address     Device I2C address
 * @return      OK in case of success; ERROR otherwise
 */
int8_t sht30_init(SHT30_t* dev, uint8_t address) {
    /* Check if the device is available */
    if(i2c_is_available(address) != I2C_RET_OK) {
        /* Device not available at given address -> return ERROR */
        return SHT30_RET_ERROR_NODEV;
    }
    /* Store address in device structure */
    dev->address = address;
    /* Set the sensor initially to not-running */
    dev->running = 0;
    /* Return OK */
    return SHT30_RET_OK;
}


/*!
 * Soft-reset the sensor.
 *
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
int8_t sht30_reset(SHT30_t* dev) {
    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command upper byte */
    if(i2c_put(SHT30_COM_SOFT_RESET >> 8) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command lower byte */
    if(i2c_put(SHT30_COM_SOFT_RESET & 0xFF) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Return OK */
    return SHT30_RET_OK;
}


/*!
 * Turn on the sensor's heater (for testing purposes).
 *
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
int8_t sht30_heater_enable(SHT30_t* dev) {
    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command upper byte */
    if(i2c_put(SHT30_COM_HEATER_ENABLE >> 8) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command lower byte */
    if(i2c_put(SHT30_COM_HEATER_ENABLE & 0xFF) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Return OK */
    return SHT30_RET_OK;
}


/*!
 * Turn off the sensor's heater (for testing purposes).
 *
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
int8_t sht30_heater_disable(SHT30_t* dev) {
    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command upper byte */
    if(i2c_put(SHT30_COM_HEATER_DISABLE >> 8) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command lower byte */
    if(i2c_put(SHT30_COM_HEATER_DISABLE & 0xFF) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Return OK */
    return SHT30_RET_OK;
}


/*!
 * Read the sensor's status register.
 *
 * @param[in]   dev         Pointer to the device structure
 * @param[out]  status      Status variable to be filled
 * @return      OK in case of success; ERROR otherwise
 */
int8_t sht30_get_status(SHT30_t* dev, uint16_t* status) {
    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command upper byte */
    if(i2c_put(SHT30_COM_STATUS >> 8) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command lower byte */
    if(i2c_put(SHT30_COM_STATUS & 0xFF) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    
    /* Issue a I2C start with read condition */
    if(i2c_start(dev->address, I2C_READ) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Read two status bytes and perform CRC check */
    if(_read16_crc(status) != SHT30_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    
    /* Return OK */
    return SHT30_RET_OK;
}


/*!
 * Clear the sensor's status register.
 *
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
int8_t sht30_clr_status(SHT30_t* dev) {
    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command upper byte */
    if(i2c_put(SHT30_COM_CLEAR_STATUS >> 8) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command lower byte */
    if(i2c_put(SHT30_COM_CLEAR_STATUS & 0xFF) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Return OK */
    return SHT30_RET_OK;
}


/***** ONE-TIME MODE ******************/
/*!
 * Read the temperature in degree Celsius (°C) from the sensor.
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  temperature     Temperature reading in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 */
int8_t sht30_get_temperature(SHT30_t* dev, float* temperature) {
    float tmp;
    /* Call reading function */
    return sht30_get_temperature_humidity(dev, temperature, &tmp);
}


/*!
 * Read the relative humidity (%RH) from the sensor.
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  humidity        Relative humidity reading (%RH)
 * @return      OK in case of success; ERROR otherwise
 */
int8_t sht30_get_humidity(SHT30_t* dev, float* humidity) {
    float tmp;
    /* Call reading function */
    return sht30_get_temperature_humidity(dev, &tmp, humidity);
}


/*!
 * Read both temperature and humidity from the sensor.
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  temperature     Temperature reading in degree Celsius (°C)
 * @param[out]  humidity        Relative humidity reading (%RH)
 * @return      OK in case of success; ERROR otherwise
 */
int8_t sht30_get_temperature_humidity(SHT30_t* dev, float* temperature, float* humidity) {
#if SHT30_CLOCK_STRETCHING==0
    uint8_t timeout = 0;        /* Timeout counter for polling */
#endif
    uint16_t temp_raw;          /* Raw temperature value from sensor */
    uint16_t humid_raw;         /* Raw humidity value from sensor */

    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command upper byte */
    if(i2c_put(SHT30_COM_MEASURE >> 8) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command lower byte */
    if(i2c_put(SHT30_COM_MEASURE & 0xFF) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }

#if SHT30_CLOCK_STRETCHING
    /* Issue a I2C start with read condition */
    if(i2c_start(dev->address, I2C_READ) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
#else
    /* Poll every 1 ms for measurement ready */
    while(++timeout < SHT30_POLLING_MAX) {
        /* Issue a I2C start with read condition */
        if(i2c_start(dev->address, I2C_READ) == I2C_RET_OK) {
            /* Exit loop */
            break;
        }
        /* Wait some time */
        _delay_ms(SHT30_POLLING_DELAY);
    }
#endif
    /* Read two temperature bytes and perform CRC check */
    if(_read16_crc(&temp_raw) != SHT30_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Read two humidty bytes and perform CRC check */
    if(_read16_crc(&humid_raw) != SHT30_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }

    /* Calculate temperature in °C and humidity in %RH */
    *temperature = _raw2temperature(temp_raw);
    *humidity = _raw2humidity(humid_raw);

    /* Return OK */
    return SHT30_RET_OK;
}


/***** PERIODIC MODE ******************/
/*!
 * Start periodic operation.
 *
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
int8_t sht30_periodic_start(SHT30_t* dev) {
    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command upper byte */
    if(i2c_put(SHT30_COM_PERIODIC >> 8) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command lower byte */
    if(i2c_put(SHT30_COM_PERIODIC & 0xFF) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Set the sensor to running */
    dev->running = 1;
    /* Return OK */
    return SHT30_RET_OK;
}


/*!
 * Break periodic operation.
 *
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
int8_t sht30_periodic_break(SHT30_t* dev) {
    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command upper byte */
    if(i2c_put(SHT30_COM_BREAK >> 8) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command lower byte */
    if(i2c_put(SHT30_COM_BREAK & 0xFF) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Set the sensor to not-running */
    dev->running = 0;
    /* Return OK */
    return SHT30_RET_OK;
}


/*!
 * Fetch the temperature in degree Celsius (°C) from the sensor in periodic mode.
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  temperature     Temperature reading in degree Celsius (°C)
 * @return      OK in case of success; ERROR otherwise
 */
int8_t sht30_fetch_temperature(SHT30_t* dev, float* temperature) {
    float tmp;
    /* Call reading function */
    return sht30_fetch_temperature_humidity(dev, temperature, &tmp);
}


/*!
 * Fetch the relative humidity (%RH) from the sensor in periodic mode.
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  humidity        Relative humidity reading (%RH)
 * @return      OK in case of success; ERROR otherwise
 */
int8_t sht30_fetch_humidity(SHT30_t* dev, float* humidity) {
    float tmp;
    /* Call reading function */
    return sht30_fetch_temperature_humidity(dev, &tmp, humidity);
}


/*!
 * Read both temperature and humidity from the sensor in periodic mode.
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  temperature     Temperature reading in degree Celsius (°C)
 * @param[out]  humidity        Relative humidity reading (%RH)
 * @return      OK in case of success; ERROR otherwise
 */
int8_t sht30_fetch_temperature_humidity(SHT30_t* dev, float* temperature, float* humidity) {
    uint16_t temp_raw;          /* Raw temperature value from sensor */
    uint16_t humid_raw;         /* Raw humidity value from sensor */

    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command upper byte */
    if(i2c_put(SHT30_COM_FETCH >> 8) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Write command lower byte */
    if(i2c_put(SHT30_COM_FETCH & 0xFF) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }

    /* Issue a I2C start with read condition */
    if(i2c_start(dev->address, I2C_READ) != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Read two temperature bytes and perform CRC check */
    if(_read16_crc(&temp_raw) != SHT30_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Read two humidty bytes and perform CRC check */
    if(_read16_crc(&humid_raw) != SHT30_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return SHT30_RET_ERROR;
    }

    /* Calculate temperature in °C and humidity in %RH */
    *temperature = _raw2temperature(temp_raw);
    *humidity = _raw2humidity(humid_raw);

    /* Return OK */
    return SHT30_RET_OK;
}
