/*!
 * @brief   ASN(x) ZMOD4410 gas sensor library -- source file
 *
 * Library to support the ZMOD4410 gas sensor.
 *
 * @file    /_asnx_lib_/sensors/zmod4410.c
 * @author  Dominik Widhalm
 * @version 1.0.0
 * @date    2022/04/14
 */


/***** INCLUDES *******************************************************/
#include "zmod4410.h"


/***** GLOBAL VARIABLES ***********************************************/
/*! Init configuration data */
uint8_t conf_init_data[] = {
    0x00, 0x50, 0x00, 0x28, 0xC3,
    0xE3, 0x00, 0x00, 0x80, 0x40,
    0x00, 0x00, 0x00, 0x00
};
/*! Init configuration structure */
ZMOD4410_CONF_t conf_init = {
    .h = {.addr = ZMOD4410_DATA_H_ADDR, .len = 2, .data_buf = &conf_init_data[0]},
    .d = {.addr = ZMOD4410_DATA_D_ADDR, .len = 2, .data_buf = &conf_init_data[2]},
    .m = {.addr = ZMOD4410_DATA_M_ADDR, .len = 2, .data_buf = &conf_init_data[4]},
    .s = {.addr = ZMOD4410_DATA_S_ADDR, .len = 4, .data_buf = &conf_init_data[6]},
    .r = {.addr = ZMOD4410_DATA_R_ADDR, .len = 4, .data_buf = &conf_init_data[10]},
};
/*! Measurement configuration data */
uint8_t conf_meas_data[] = {
    0x00, 0x50, 0xFF, 0x38, 0xFE,
    0xD4, 0xFE, 0x70, 0xFE, 0x0C,
    0xFD, 0xA8, 0xFD, 0x44, 0xFC,
    0xE0, 0x00, 0x52, 0x02, 0x67,
    0x00, 0xCD, 0x03, 0x34, 0x23,
    0x03, 0xA3, 0x43, 0x00, 0x00,
    0x06, 0x49, 0x06, 0x4A, 0x06,
    0x4B, 0x06, 0x4C, 0x06, 0x4D,
    0x06, 0x4E, 0x06, 0x97, 0x06,
    0xD7, 0x06, 0x57, 0x06, 0x4E,
    0x06, 0x4D, 0x06, 0x4C, 0x06,
    0x4B, 0x06, 0x4A, 0x86, 0x59,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00
};
/*! Measurement configuration structure */
ZMOD4410_CONF_t conf_meas = {
    .h = {.addr = ZMOD4410_DATA_H_ADDR, .len = 16, .data_buf = &conf_meas_data[0]},
    .d = {.addr = ZMOD4410_DATA_D_ADDR, .len =  8, .data_buf = &conf_meas_data[16]},
    .m = {.addr = ZMOD4410_DATA_M_ADDR, .len =  4, .data_buf = &conf_meas_data[24]},
    .s = {.addr = ZMOD4410_DATA_S_ADDR, .len = 32, .data_buf = &conf_meas_data[28]},
    .r = {.addr = ZMOD4410_DATA_R_ADDR, .len = 32, .data_buf = &conf_meas_data[60]},
};


/***** LOCAL FUNCTION PROTOTYPES **************************************/
static int8_t _read_adc(ZMOD4410_t* dev, ZMOD4410_CONF_DATA_t* r_data);
static int8_t _init_sensor(ZMOD4410_t* dev);
static int8_t _init_measurement(ZMOD4410_t* dev);
static int8_t _calc_hsp(ZMOD4410_CONF_t* preset, uint8_t* config, uint8_t* hsp);
static int8_t _calc_iaq(uint8_t* raw, ZMOD4410_DATA_t* data);


/***** FUNCTIONS ******************************************************/

/***** GENERAL ************************/
/*!
 * Initialization of a ZMOD4410 sensor instance.
 * 
 * @param[out]  dev         Pointer to the device structure to be filled
 * @param[in]   address     Device I2C address
 * @return      OK in case of success; ERROR otherwise
 */
int8_t zmod4410_init(ZMOD4410_t* dev, uint8_t address) {
    /* Check if the device is available */
    if(i2c_is_available(address) != I2C_RET_OK) {
        /* Return ERROR */
        return ZMOD4410_RET_ERROR_NODEV;
    }
    /* Store address in device structure */
    dev->address = address;
    /* Set sensor specific configurations */
    dev->init = &conf_init;
    dev->meas = &conf_meas;
    /* Read sensor configuration */
    if(zmod4410_get_config(dev, dev->config) != ZMOD4410_RET_OK) {
        /* Reading the config failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Read sensor production data */
    if(zmod4410_get_production_data(dev, dev->prod_data) != ZMOD4410_RET_OK) {
        /* Reading the production data failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Initialize the sensor */
    if(_init_sensor(dev) != ZMOD4410_RET_OK) {
        /* Initializing the sensor failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Initialize the sensor for measurement */
    if(_init_measurement(dev) != ZMOD4410_RET_OK) {
        /* Initializing the sensor for measurement failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Return with success */
    return ZMOD4410_RET_OK;
}


/*!
 * Read the status value from the ZMOD4410 sensor.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[out]  status      Pointer to the status variable to be filled
 * @return      OK in case of success; ERROR otherwise
 */
int8_t zmod4410_get_status(ZMOD4410_t* dev, uint8_t* status) {
    /* Read the status register */
    if(i2c_read_U8(dev->address, ZMOD4410_ADDR_STATUS, status) != I2C_RET_OK) {
        /* Reading failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Return with success */
    return ZMOD4410_RET_OK;
}


/*!
 * Read the PID from the ZMOD4410 sensor.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[out]  pid         Pointer to the PID variable to be filled
 * @return      OK in case of success; ERROR otherwise
 */
int8_t zmod4410_get_pid(ZMOD4410_t* dev, uint16_t* pid) {
    /* Read the PID register */
    if(i2c_read_U16BE(dev->address, ZMOD4410_ADDR_PID, pid) != I2C_RET_OK) {
        /* Reading failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Return with success */
    return ZMOD4410_RET_OK;
}


/*!
 * Read the configuration from the ZMOD4410 sensor (6 bytes).
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[out]  conf        Pointer to the start of the array to be filled
 * @return      OK in case of success; ERROR otherwise
 */
int8_t zmod4410_get_config(ZMOD4410_t* dev, uint8_t* conf) {
    /* Read the configuration register */
    if(i2c_read_block(dev->address, ZMOD4410_ADDR_CONF, conf, 6) != I2C_RET_OK) {
        /* Reading failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Return with success */
    return ZMOD4410_RET_OK;
}


/*!
 * Read the production data from the ZMOD4410 sensor (7 bytes).
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[out]  data        Pointer to the start of the array to be filled
 * @return      OK in case of success; ERROR otherwise
 */
int8_t zmod4410_get_production_data(ZMOD4410_t* dev, uint8_t* data) {
    /* Read the production data register */
    if(i2c_read_block(dev->address, ZMOD4410_ADDR_DATA, data, 7) != I2C_RET_OK) {
        /* Reading failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Return with success */
    return ZMOD4410_RET_OK;
}


/*!
 * Read the error register value from the ZMOD4410 sensor.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[out]  error       Pointer to the error variable to be filled
 * @return      OK in case of success; ERROR otherwise
 */
int8_t zmod4410_get_error(ZMOD4410_t* dev, uint8_t* error) {
    /* Read the error register */
    if(i2c_read_U8(dev->address, ZMOD4410_ADDR_ERROR, error) != I2C_RET_OK) {
        /* Reading failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Return with success */
    return ZMOD4410_RET_OK;
}


/***** MEASUREMENT ********************/
/*!
 * Query the start of the measurements of the ZMOD4410 sensor.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
int8_t zmod4410_start(ZMOD4410_t* dev) {
    /* Write start command to the data register */
    if(i2c_write_8(dev->address, ZMOD4410_ADDR_CMD, ZMOD4410_DATA_START) != I2C_RET_OK) {
        /* Writing failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Return with success */
    return ZMOD4410_RET_OK;
}


/*!
 * Query the stop of the measurements of the ZMOD4410 sensor.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
int8_t zmod4410_stop(ZMOD4410_t* dev) {
    /* Write start command to the data register */
    if(i2c_write_8(dev->address, ZMOD4410_ADDR_CMD, ZMOD4410_DATA_STOP) != I2C_RET_OK) {
        /* Writing failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Return with success */
    return ZMOD4410_RET_OK;
}


/*!
 * Get the measurements of the ZMOD4410 sensor.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[out]  data        Pointer to the measurement data structure to be filled
 * @return      OK in case of success; ERROR otherwise
 */
int8_t zmod4410_get_measurement(ZMOD4410_t* dev, ZMOD4410_DATA_t* data) {
    /* Status register value */
    uint8_t status;
    /* Retry counter for timeout */
    uint8_t tries = 0;
    
    /* Write the measurement start command */
    if(zmod4410_start(dev) != ZMOD4410_RET_OK) {
        /* Writing start command failed */
        return ZMOD4410_RET_ERROR;
    }
    
    /* Wait for the sensor to finish */
    do {
        /* Read the status */
        if(zmod4410_get_status(dev, &status) != ZMOD4410_RET_OK) {
            /* Reading the status failed */
            return ZMOD4410_RET_ERROR;
        }
        /* Check if status is "running" */
        if (!(status & ZMOD4410_STATUS_SEQ_RUNNING)) {
            /* DONE */
            break;
        }
        /* Increase retry counter */
        tries++;
        /* Delay 50ms */
        _delay_ms(50);
    } while(tries < ZMOD4410_POLLING_MAX);
    /* Check if timeout was reached */
    if(tries >= ZMOD4410_POLLING_MAX) {
        /* Reading from sensor timed out */
        return ZMOD4410_RET_ERROR_TIMEOUT;
    }
    
    /* Write the measurement stop command */
    if(zmod4410_stop(dev) != ZMOD4410_RET_OK) {
        /* Writing stop command failed */
        return ZMOD4410_RET_ERROR;
    }
    
    /* Read ADC data */
    if(_read_adc(dev, &(dev->meas->r)) != ZMOD4410_RET_OK) {
        /* Reading the ADC failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Calculate IAQ data from ADC data */
    if(_calc_iaq(dev->init->r.data_buf, data) != 0) {
        /* Calculation failed */
        return ZMOD4410_RET_ERROR_CALCULATION;
    }
    /* Return with success */
    return ZMOD4410_RET_OK;
}


/***** LOCAL **************************/
/*!
 * Read ADC values from the ZMOD4410 sensor into the R data set.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @param[out]  r_data      Pointer to the R data structure to be filled
 * @return      OK in case of success; ERROR otherwise
 */
static int8_t _read_adc(ZMOD4410_t* dev, ZMOD4410_CONF_DATA_t* r_data) {
    /* Read the R data set register */
    if(i2c_read_block(dev->address, r_data->addr, r_data->data_buf, r_data->len) != I2C_RET_OK) {
        /* Reading failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Return with success */
    return ZMOD4410_RET_OK;
}


/*!
 * Basic initialization of the sensor (e.g., after power-on).
 * 
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
static int8_t _init_sensor(ZMOD4410_t* dev) {
    /* Status register value */
    uint8_t status;
    /* Intermediate HSP value */
    uint8_t hsp[16] = {0};
    /* Retry counter for timeout */
    uint8_t tries = 0;
    
    /* Calculate the init HSP factor */
    if(_calc_hsp(dev->init, dev->config, hsp) != ZMOD4410_RET_OK) {
        /* Calculating the factor failed */
        return ZMOD4410_RET_ERROR;
    }
    
    /* Write H data */
    if(i2c_write_block(dev->address, dev->init->h.addr, hsp, dev->init->h.len) != I2C_RET_OK) {
        /* Writing H register failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Write D data */
    if(i2c_write_block(dev->address, dev->init->d.addr, dev->init->d.data_buf, dev->init->d.len) != I2C_RET_OK) {
        /* Writing D register failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Write M data */
    if(i2c_write_block(dev->address, dev->init->m.addr, dev->init->m.data_buf, dev->init->m.len) != I2C_RET_OK) {
        /* Writing M register failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Write S data */
    if(i2c_write_block(dev->address, dev->init->s.addr, dev->init->s.data_buf, dev->init->s.len) != I2C_RET_OK) {
        /* Writing S register failed */
        return ZMOD4410_RET_ERROR;
    }
    
    /* Write the measurement start command */
    if(zmod4410_start(dev) != ZMOD4410_RET_OK) {
        /* Writing start command failed */
        return ZMOD4410_RET_ERROR;
    }
    
    /* Wait for the sensor to finish */
    do {
        /* Read the status */
        if(zmod4410_get_status(dev, &status) != ZMOD4410_RET_OK) {
            /* Reading the status failed */
            return ZMOD4410_RET_ERROR;
        }
        /* Check if status is "running" */
        if (!(status & ZMOD4410_STATUS_SEQ_RUNNING)) {
            /* DONE */
            break;
        }
        /* Increase retry counter */
        tries++;
        /* Delay 50ms */
        _delay_ms(50);
    } while(tries < ZMOD4410_POLLING_MAX);
    /* Check if timeout was reached */
    if(tries >= ZMOD4410_POLLING_MAX) {
        /* Reading from sensor timed out */
        return ZMOD4410_RET_ERROR_TIMEOUT;
    }
    
    /* Read R data */
    if(i2c_read_block(dev->address, dev->init->r.addr, dev->init->r.data_buf, dev->init->r.len) != I2C_RET_OK) {
        /* Reading R data failed */
        return ZMOD4410_RET_ERROR;
    }
    
    /* Write the measurement stop command */
    if(zmod4410_stop(dev) != ZMOD4410_RET_OK) {
        /* Writing stop command failed */
        return ZMOD4410_RET_ERROR;
    }
    
    /* Set sensor specific parameters */
    dev->mox_lr = (uint16_t)((uint16_t)(dev->init->r.data_buf[0] << 8) | dev->init->r.data_buf[1]);
    dev->mox_er = (uint16_t)((uint16_t)(dev->init->r.data_buf[2] << 8) | dev->init->r.data_buf[3]);
    
    /* Return with success */
    return ZMOD4410_RET_OK;
}


/*!
 * Initialize the sensor for corresponding measurement.
 * 
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
static int8_t _init_measurement(ZMOD4410_t* dev) {
    /* Intermediate HSP value */
    uint8_t hsp[16] = {0};

    /* Calculate the measurement HSP factor */
    if(_calc_hsp(dev->meas, dev->config, hsp) != ZMOD4410_RET_OK) {
        /* Calculating the factor failed */
        return ZMOD4410_RET_ERROR;
    }
    
    /* Write H data */
    if(i2c_write_block(dev->address, dev->meas->h.addr, hsp, dev->meas->h.len) != I2C_RET_OK) {
        /* Writing H register failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Write D data */
    if(i2c_write_block(dev->address, dev->meas->d.addr, dev->meas->d.data_buf, dev->meas->d.len) != I2C_RET_OK) {
        /* Writing D register failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Write M data */
    if(i2c_write_block(dev->address, dev->meas->m.addr, dev->meas->m.data_buf, dev->meas->m.len) != I2C_RET_OK) {
        /* Writing M register failed */
        return ZMOD4410_RET_ERROR;
    }
    /* Write S data */
    if(i2c_write_block(dev->address, dev->meas->s.addr, dev->meas->s.data_buf, dev->meas->s.len) != I2C_RET_OK) {
        /* Writing S register failed */
        return ZMOD4410_RET_ERROR;
    }
    
    /* Return with success */
    return ZMOD4410_RET_OK;
}


/*!
 * Calculate the measurement settings (hsp value) of the ZMOD4410 sensor.
 * 
 * @param[in]   preset      Pointer to the default configuration structure
 * @param[in]   config      Pointer to the instance configuration structure
 * @param[out]  hsp         Pointer to the HSP data variable to be written
 * @return      OK in case of success; ERROR otherwise
 */
static int8_t _calc_hsp(ZMOD4410_CONF_t* preset, uint8_t* config, uint8_t* hsp) {
    int16_t hsp_temp[8] = {0};
    float   hspf = 0.0;
    uint8_t i = 0;
    /* Iterate over the H data set elements */
    while(i < preset->h.len) {
        /* Calculate intermediate hsp value */
        hsp_temp[i/2] = (int16_t)(((uint16_t)(preset->h.data_buf[i]) << 8) + (preset->h.data_buf[i+1]));
        hspf = (-((float)(config[2]) * 256.0F + config[3]) * ((config[4] + 640.0F) * (float)(config[5] + hsp_temp[i/2]) - 512000.0F)) / 12288000.0F;
        /* Calculate H data set elements based on intermediate value */
        hsp[i] = (uint8_t)((uint16_t)(hspf) >> 8);
        hsp[i+1] = (uint8_t) ((uint16_t)(hspf) & 0x00FF);
        /* Increment data set index by 2 */
        i += 2;
    }
    /* Return with success */
    return ZMOD4410_RET_OK;
}


/*!
 * Calculate the sensor data from the raw measurements.
 * 
 * @param[in]   raw         Pointer to the raw data structure
 * @param[out]  data        Pointer to the sensor data structure to be written
 * @return      OK in case of success; ERROR otherwise
 */
static int8_t _calc_iaq(uint8_t* raw, ZMOD4410_DATA_t* data) {
    // TODO: "calc_iaq_2nd_gen" looks like closed source; not in the repo :/
    
    // TODO: silence warning for now ...
    (void)raw;
    
    // TODO: fill data with zeros for now ...
    for(uint8_t i=0; i<13; i++) {
        data->rmox[i] = 0.0;
    }
    data->log_rcda = 0.0;
    data->iaq = 0.0;
    data->tvoc = 0.0;
    data->etoh = 0.0;
    data->eco2 = 0.0;
    
    /* Return with success */
    return ZMOD4410_RET_OK;
}
