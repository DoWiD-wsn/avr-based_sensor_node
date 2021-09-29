/*!
 * @brief   ASN(x) SHTC3 sensor library -- source file
 *
 * Library to support the SHTC3 sensor (via I2C).
 *
 * @file    /_asnx_lib_/sensors/shtc3.c
 * @author  Dominik Widhalm
 * @version 1.1.0
 * @date    2021/09/28
 * 
 * @see     https://github.com/Sensirion/shtc3-stm-sample-project/
 * @note    Based on the SHTC3 sample code for STM32 by Sensirion
 * @todo    Clock-stretching did not work in combination with sleep
 */


/***** INCLUDES *******************************************************/
#include "shtc3.h"


/***** LOCAL FUNCTION PROTOTYPE ***************************************/
SHTC3_RET_t shtc3_read16_crc(uint16_t* value);
static SHTC3_RET_t shtc3_check_crc(uint8_t* data, uint8_t len, uint8_t checksum);

static float SHTC3_raw2temperature(uint16_t raw);
static float SHTC3_raw2humidity(uint16_t raw);


/***** FUNCTIONS ******************************************************/
/*!
 * Initialization of the SHTC3 sensor.
 *
 * @param[out]  dev         Pointer to the device structure to be filled
 * @param[in]   address     Device I2C address
 * @return      OK in case of success; ERROR otherwise
 */
SHTC3_RET_t shtc3_init(SHTC3_t* dev, uint8_t address) {
   /* Check if the device is available */
   if(i2c_is_available(address) == I2C_RET_OK) {
        /* Device is available ... store address in device structure */
        dev->address = address;
        /* Return OK */
        return SHTC3_RET_OK;
   } else {
        /* Return ERROR */
        return SHTC3_RET_ERROR_NODEV;
   }
}


/*!
 * Read 16-bit from the SHTC3 sensor and perform CRC check.
 *
 * @param[out]  value       Pointer to the location of the 16-bit variable to be read
 * @return      OK in case of success; ERROR otherwise
 */
SHTC3_RET_t shtc3_read16_crc(uint16_t* value) {
    /* Temporary data bytes */
    uint8_t data[2];
    /* Checksum byte */
    uint8_t checksum;
    
    /* Read two data bytes and one checksum byte */
    if(i2c_get_ack(&data[0]) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    if(i2c_get_ack(&data[1]) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    if(i2c_get_ack(&checksum) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    
    /* Verify checksum */
    if(shtc3_check_crc(data, 2, checksum) != SHTC3_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR_CRC;
    }
    /* Combine the two bytes to a 16-bit value */
    *value = (data[0] << 8) | data[1];
    
    /* Return OK */
    return SHTC3_RET_OK;
}


/*!
 * Perform a CRC check.
 *
 * @param[in]   data        Input data to be checked
 * @param[in]   len         Number of bytes to be checked
 * @param[in]   checksum    Received CRC checksum
 * @return      OK in case of success; ERROR otherwise
 */
static SHTC3_RET_t shtc3_check_crc(uint8_t* data, uint8_t len, uint8_t checksum) {
    uint8_t bit;            /* bit mask */
    uint8_t crc = 0xFF;     /* Calculated checksum */
    uint8_t cnt;            /* byte counter */

    /* Calculate CRC with given polynomial */
    for(cnt=0; cnt<len; cnt++) {
        crc ^= (data[cnt]);
        for(bit=8; bit>0; --bit) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ SHTC3_CRC_POLYNOMIAL;
            } else {
                crc = (crc << 1);
            }
        }
    }

    /* Verify checksum */
    if(crc != checksum) {
        return SHTC3_RET_ERROR_CRC;
    } else {
        return SHTC3_RET_OK;
    }
}


/*!
 * Soft-reset the sensor.
 *
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
SHTC3_RET_t shtc3_reset(SHTC3_t* dev) {
    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    
    /* Write ID read command upper byte */
    if(i2c_put(SHTC3_COM_SOFT_RESET >> 8) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    /* Write ID read command lower byte */
    if(i2c_put(SHTC3_COM_SOFT_RESET & 0xFF) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }

    /* Return OK */
    return SHTC3_RET_OK;
}


/*!
 * Read the SHTC3 sensor's ID.
 *
 * @param[in]   dev         Pointer to the device structure
 * @param[out]  id          Location where the ID should be stored
 * @return      OK in case of success; ERROR otherwise
 */
SHTC3_RET_t shtc3_get_id(SHTC3_t* dev, uint16_t *id) {
    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    
    /* Write ID read command upper byte */
    if(i2c_put(SHTC3_COM_READID >> 8) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    /* Write ID read command lower byte */
    if(i2c_put(SHTC3_COM_READID & 0xFF) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    
    /* Issue a I2C start with read condition */
    if(i2c_start(dev->address, I2C_READ) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    /* Read two bytes and perform CRC check */
    if(shtc3_read16_crc(id) != SHTC3_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }

    /* Return OK */
    return SHTC3_RET_OK;
}


/*!
 * Request the SHTC3 sensor to sleep.
 *
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
SHTC3_RET_t shtc3_sleep_enable(SHTC3_t* dev) {
    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    
    /* Write ID read command upper byte */
    if(i2c_put(SHTC3_COM_SLEEP >> 8) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    /* Write ID read command lower byte */
    if(i2c_put(SHTC3_COM_SLEEP & 0xFF) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }

    /* Return OK */
    return SHTC3_RET_OK;
}


/*!
 * Request the SHTC3 sensor to wake-up.
 *
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
SHTC3_RET_t shtc3_sleep_disable(SHTC3_t* dev) {
    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    
    /* Write ID read command upper byte */
    if(i2c_put(SHTC3_COM_WAKEUP >> 8) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    /* Write ID read command lower byte */
    if(i2c_put(SHTC3_COM_WAKEUP & 0xFF) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    
    /* Give the device some time to wake-up */
    _delay_us(SHTC3_WAKEUP_DELAY);

    /* Return OK */
    return SHTC3_RET_OK;
}


/*!
 * Convert a raw temperature to degrees Celsius.
 * T = -45 + 175 * raw / 2^16
 *
 * @param[in]   raw         Raw temperature reading
 * @return      Resulting temperature in degrees Celsius
 */
static float SHTC3_raw2temperature(uint16_t raw) {
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
static float SHTC3_raw2humidity(uint16_t raw) {
  /* Calculate resulting relative humidity [%RH] */
  return 100.0 * ((float)raw / 65536.0);
}


/*!
 * Read both temperature and humidity from the sensor.
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  temperature     Temperature reading in degree Celsius (°C)
 * @param[out]  humidity        Relative humidity reading (%RH)
 * @param[in]   lp_en           Enable low-power mode (1..enabled/0..disabled)
 * @return      OK in case of success; ERROR otherwise
 */
SHTC3_RET_t shtc3_get_temperature_humidity(SHTC3_t* dev, float* temperature, float* humidity, uint8_t lp_en) {
#if SHTC3_CLOCK_STRETCHING==0
    uint8_t timeout = 0;        /* Timeout counter for polling */
#endif
    uint16_t temp_raw;          /* Raw temperature value from sensor */
    uint16_t humid_raw;         /* Raw humidity value from sensor */
    /* Measurement command depends on mode setting */
    uint16_t cmd_meas = lp_en ? SHTC3_COM_MEAS_LPOW : SHTC3_COM_MEAS_NORM;

    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    
    /* Write read command upper byte */
    if(i2c_put(cmd_meas >> 8) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    /* Write read command lower byte */
    if(i2c_put(cmd_meas & 0xFF) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    
#if SHTC3_CLOCK_STRETCHING
    /* Issue a I2C start with read condition */
    if(i2c_start(dev->address, I2C_READ) != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
#else
    /* Poll every 1 ms for measurement ready */
    while(++timeout < SHTC3_POLLING_MAX) {
        /* Issue a I2C start with read condition */
        if(i2c_start(dev->address, I2C_READ) == I2C_RET_OK) {
            /* Exit loop */
            break;
        }
        /* Wait some time */
        _delay_ms(SHTC3_POLLING_DELAY);
    }
#endif

#if SHTC3_RH_FIRST
    /* Read two humidty bytes and perform CRC check */
    if(shtc3_read16_crc(&humid_raw) != SHTC3_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    /* Read two temperature bytes and perform CRC check */
    if(shtc3_read16_crc(&temp_raw) != SHTC3_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
#else
    /* Read two temperature bytes and perform CRC check */
    if(shtc3_read16_crc(&temp_raw) != SHTC3_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
    /* Read two humidty bytes and perform CRC check */
    if(shtc3_read16_crc(&humid_raw) != SHTC3_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }
#endif
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return SHTC3_RET_ERROR;
    }

    /* Calculate temperature in °C and humidity in %RH */
    *temperature = SHTC3_raw2temperature(temp_raw);
    *humidity = SHTC3_raw2humidity(humid_raw);

    /* Return OK */
    return SHTC3_RET_OK;
}


/*!
 * Read the temperature in degree Celsius (°C) from the sensor.
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  temperature     Temperature reading in degree Celsius (°C)
 * @param[in]   lp_en           Enable low-power mode (1..enabled/0..disabled)
 * @return      OK in case of success; ERROR otherwise
 */
SHTC3_RET_t shtc3_get_temperature(SHTC3_t* dev, float* temperature, uint8_t lp_en) {
    float tmp;
    /* Call reading function */
    return shtc3_get_temperature_humidity(dev, temperature, &tmp, lp_en);
}


/*!
 * Read the relative humidity (%RH) from the sensor.
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  humidity        Relative humidity reading (%RH)
 * @param[in]   lp_en           Enable low-power mode (1..enabled/0..disabled)
 * @return      OK in case of success; ERROR otherwise
 */
SHTC3_RET_t shtc3_get_humidity(SHTC3_t* dev, float* humidity, uint8_t lp_en) {
    float tmp;
    /* Call reading function */
    return shtc3_get_temperature_humidity(dev, &tmp, humidity, lp_en);
}


/*!
 * Initialize the sensor and put it to sleep.
 * 
 *
 * @param[out]  dev         Pointer to the device structure
 * @param[in]   address     Device I2C address
 * @return      OK in case of success; ERROR otherwise
 */
SHTC3_RET_t shtc3_start(SHTC3_t* dev, uint8_t address) {
    int8_t ret;
    /* Initialize the sensor */
    ret = shtc3_init(dev, address);
    if(ret != SHTC3_RET_OK) {
        return ret;
    }
    /* Put the sensor to sleep */
    ret = shtc3_sleep_enable(dev);
    if(ret != SHTC3_RET_OK) {
        return ret;
    }
    /* Return OK */
    return SHTC3_RET_OK;
}


/*!
 * Read both temperature and humidity from the sensor (incl. sleep/wakeup).
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  temperature     Temperature reading in degree Celsius (°C)
 * @param[out]  humidity        Relative humidity reading (%RH)
 * @param[in]   lp_en           Enable low-power mode (1..enabled/0..disabled)
 * @return      OK in case of success; ERROR otherwise
 */
SHTC3_RET_t shtc3_read(SHTC3_t* dev, float* temperature, float* humidity, uint8_t lp_en) {
    int8_t ret;
    /* Wake-up the sensor */
    ret = shtc3_sleep_disable(dev);
    if(ret != SHTC3_RET_OK) {
        return ret;
    }
    /* Read measurements */
    ret = shtc3_get_temperature_humidity(dev, temperature, humidity, lp_en);
    if(ret != SHTC3_RET_OK) {
        return ret;
    }
    /* Put the sensor to sleep */
    ret = shtc3_sleep_enable(dev);
    if(ret != SHTC3_RET_OK) {
        return ret;
    }
    /* Return OK */
    return SHTC3_RET_OK;
}
