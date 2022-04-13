/*!
 * @brief   ASN(x) BH1750 sensor library -- source file
 *
 * Library to support the BH1750 sensor (via I2C).
 *
 * @file    /_asnx_lib_/sensors/bh1750.c
 * @author  Dominik Widhalm
 * @version 1.0.0
 * @date    2022/04/13
 */


/***** INCLUDES *******************************************************/
#include "bh1750.h"


/***** FUNCTIONS ******************************************************/
/*!
 * Initialization of the BH1750 sensor with default address and DVI GPIO.
 *
 * @param[out]  dev         Pointer to the device structure to be filled
 * @return      OK in case of success; ERROR otherwise
 */
int8_t bh1750_init_default(BH1750_t* dev) {
    /* Call general init function with default values */
    return bh1750_init(dev, BH1750_I2C_ADDRESS, &BH1750_DVI_DDR, &BH1750_DVI_PORT, &BH1750_DVI_PIN, BH1750_DVI_GPIO);
}


/*!
 * Initialization of the BH1750 sensor.
 *
 * @param[out]  dev         Pointer to the device structure to be filled
 * @param[in]   address     Device I2C address
 * @param[in]   ddr         Pointer to the GPIO's DDRx register
 * @param[in]   port        Pointer to the GPIO's PORTx register
 * @param[in]   pin         Pointer to the GPIO's PINx register
 * @param[in]   portpin     Index of the GPIO pin
 * @return      OK in case of success; ERROR otherwise
 */
int8_t bh1750_init(BH1750_t* dev, uint8_t address, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin) {
    /* Get a GPIO handle for the DVI pin */
    hw_get_io(&(dev->dvi), ddr, port, pin, portpin);
    /* Get a pointer to the DVI GPIO structure */
    hw_io_t* dvi = &(dev->dvi);
    /* Set the DVI pin to output */
    HW_GPIO_OUTPUT(dvi);
    /* Perform asynchronous reset via DVI pin (necessary after start-up) */
    bh1750_reset_async(dev);
    /* Check if the device is available */
    if(i2c_is_available(address) != I2C_RET_OK) {
        /* Device not available at given address -> return ERROR */
        return BH1750_RET_ERROR_NODEV;
    }
    /* Store address in device structure */
    dev->address = address;
    /* Return OK */
    return BH1750_RET_OK;
}


/*!
 * Request power down (sleep) mode.
 *
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
int8_t bh1750_sleep_enable(BH1750_t* dev) {
    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    /* Write Opecode byte */
    if(i2c_put(BH1750_COM_POWER_DOWN) != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    /* Return OK */
    return BH1750_RET_OK;
}


/*!
 * Request power on (active) mode.
 *
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
int8_t bh1750_sleep_disable(BH1750_t* dev) {
    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    /* Write Opecode byte */
    if(i2c_put(BH1750_COM_POWER_ON) != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    /* Return OK */
    return BH1750_RET_OK;
}


/*!
 * Request device reset.
 *
 * @param[in]   dev         Pointer to the device structure
 * @return      OK in case of success; ERROR otherwise
 */
int8_t bh1750_reset(BH1750_t* dev) {
    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    /* Write Opecode byte */
    if(i2c_put(BH1750_COM_RESET) != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    /* Return OK */
    return BH1750_RET_OK;
}


/*!
 * Perform an asynchronous reset via the DVI pin.
 *
 * @param[in]   dev         Pointer to the device structure
 */
void bh1750_reset_async(BH1750_t* dev) {
    /* Get a pointer to the DVI GPIO structure */
    hw_io_t* dvi = &(dev->dvi);
    /* First, have the DVI pin at low level */
    HW_GPIO_LOW(dvi);
    /* Wait for a defined delay */
    _delay_us(BH1750_RESET_DELAY);
    /* Pull the DVI pin high */
    HW_GPIO_HIGH(dvi);
    /* All done */
    return;
}


/*!
 * Read the illuminance value from the sensor.
 *
 * @param[in]   dev         Pointer to the device structure
 * @param[out]  illuminance Pointer to the data variable to be written
 * @return      OK in case of success; ERROR otherwise
 */
int8_t bh1750_get_illuminance(BH1750_t* dev, uint16_t* illuminance) {
    /* Temporary data bytes */
    uint8_t data[2];
    /* Issue a I2C start with write condition */
    if(i2c_start(dev->address, I2C_WRITE) != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    /* Write measurement command byte */
    if(i2c_put(BH1750_MEASURE) != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    
    /* Wait for the conversion to be completed */
    _delay_ms(BH1750_MEASURE_DELAY_MS);
    
    /* Issue a I2C start with read condition */
    if(i2c_start(dev->address, I2C_READ) != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    /* Read two data bytes */
    if(i2c_get_ack(&data[1]) != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    if(i2c_get_ack(&data[0]) != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    /* Issue a I2C stop condition */
    if(i2c_stop() != I2C_RET_OK) {
        /* Return ERROR */
        return BH1750_RET_ERROR;
    }
    
    /* Convert raw reading to illuminance */
    *illuminance = (uint16_t)((data[0] | ((uint16_t)data[1]<<8)) / 1.2);
    
    /* Return OK */
    return BH1750_RET_OK;
}
