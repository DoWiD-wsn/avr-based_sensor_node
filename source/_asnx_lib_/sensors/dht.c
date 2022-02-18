/*!
 * @brief   ASN(x) DHT temperature/humidity sensor library -- source file
 *
 * Library to support the DHT temperature/humidity sensor.
 * The low level reading is partly based on the code of Davide Gironi.
 *
 * @file    /_asnx_lib_/sensors/dht.c
 * @author  Dominik Widhalm
 * @version 1.2.6
 * @date    2022/02/18
 *
 * @see     http://davidegironi.blogspot.com/2013/02/reading-temperature-and-humidity-on-avr.html
 */


/***** INCLUDES *******************************************************/
#include "dht.h"


/***** LOCAL FUNCTION PROTOTYPES **************************************/
static DHT_RET_t _read(DHT_t* dev);


/***** FUNCTIONS **************************************************************/
/*!
 * Initialization of a DHT sensor instance.
 * 
 * @param[out]  dev     Pointer to the device structure to be filled
 * @param[in]   ddr     Pointer to the GPIO's DDRx register
 * @param[in]   port    Pointer to the GPIO's PORTx register
 * @param[in]   pin     Pointer to the GPIO's PINx register
 * @param[in]   portpin Index of the GPIO pin
 * @param[in]   type    DHT sensor type
 * @return      OK* in case of success; ERROR* otherwise
 */
DHT_RET_t dht_init(DHT_t* dev, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin, DHT_DEV_t type) {
    /* Fill the HW GPIO structure */
    dev->gpio.ddr = ddr;
    dev->gpio.port = port;
    dev->gpio.pin = pin;
    dev->gpio.portpin = portpin;
    /* Set the sensor type */
    dev->type = type;
    /* Get a pointer to the HW GPIO structure */
    hw_io_t* gpio = &(dev->gpio);
    /* Setup the GPIO pin */
    HW_GPIO_OUTPUT(gpio);
    HW_GPIO_HIGH(gpio);
    /* Init was successful */
    return DHT_RET_OK;
}


/*!
 * Perform a low-level reading from the sensor.
 * 
 * @param[in]   dev     Pointer to the device structure
 * @return      OK* in case of success; ERROR* otherwise
 */
DHT_RET_t dht_reset(DHT_t* dev) {
    /* Get a pointer to the HW GPIO structure */
    hw_io_t* gpio = &(dev->gpio);
    /* Reset the data line */
    HW_GPIO_OUTPUT(gpio);
    HW_GPIO_HIGH(gpio);
    /* Give the sensor some time to reset */
    _delay_ms(100);
    /* Return */
    return DHT_RET_OK;
}

/*!
 * Perform a low-level reading from the sensor.
 * 
 * @param[in]   dev     Pointer to the device structure
 * @return      OK* in case of success; ERROR* otherwise
 */
static DHT_RET_t _read(DHT_t* dev) {
    /* Get a pointer to the HW GPIO structure */
    hw_io_t* gpio = &(dev->gpio);
    /* Temporary counter variables */
    uint8_t i,j = 0;
    
#if DHT_CHECK_LAST_MEAS
    /*** Check if sensor was read less than two seconds ago ***/
    static uint32_t lasttime = 0;
    /* Read in the current tick count (ms) */
    uint32_t currenttime = systick_get_ticks();
    /* Check if there was already a previous reading */
    if(lasttime != 0) {
        /* Check if at least 2 seconds have elapsed) */
        if((currenttime - lasttime) < DHT_TIMING_MIN_INTERVAL) {
            /* Too less time, use old measurement result */
            return DHT_RET_OK_LAST_VALUE;
        }
    }
    /* Update last reading time */
    lasttime = currenttime;
#endif
    
    /* Perform a new read -> reset result data section */
    for(i=0; i<5; i++) {
        dev->data[i] = 0;
    }
    
    /*** Start timing-critical section ***/
    /* Disable interrupts */
    cli();

    /* Send a read request */
    HW_GPIO_LOW(gpio);
    /* And wait for a period according to sensor type */
    switch(dev->type) {
        case DHT_DEV_DHT21:
        case DHT_DEV_DHT22:
            /* Datasheet says "at least 1ms", 1.1ms just to be safe */
            _delay_us(1100);
            break;
        case DHT_DEV_DHT11:
        case DHT_DEV_DHT12:
            /* Datasheet says "at least 18ms", 20ms just to be safe */
            _delay_ms(18);
            break;
        default:
            /* Re-enable interrupts */
            sei();
            /* Unsupported sensor type */
            return DHT_RET_ERROR_TYPE;
    }
    /* Release the data line */
    HW_GPIO_HIGH(gpio);
    HW_GPIO_INPUT(gpio);
    _delay_us(40);
    
    /* Check start condition high level */
    if(HW_GPIO_READ(gpio)) {
        /* Re-enable interrupts */
        sei();
        /* Did not get start condition */
        return DHT_RET_ERROR_START;
    }
    _delay_us(80);
    /* Check start condition low level */
    if(!HW_GPIO_READ(gpio)) {
        /* Re-enable interrupts */
        sei();
        /* Did not get start condition */
        return DHT_RET_ERROR_START;
    }
    _delay_us(80);
    
    uint16_t cnt = 0;
    /* Read the data send by the sensor (should be 5 byte) ... */
    for(j=0; j<5; j++) {
        uint8_t result=0;
        /* ... bit by bit */
        for(i=0; i<8; i++) {
            /* Reset timeout counter */
            cnt = 0;
            /* Wait for the data line to become high */
            while(!HW_GPIO_READ(gpio)) {
                cnt++;
                /* Check if timeout has been reached */
                if(cnt > DHT_TIMING_TIMEOUT) {
                    /* Re-enable interrupts */
                    sei();
                    /* Timeout reached */
                    return DHT_RET_ERROR_TIMEOUT;
                }
            }
            _delay_us(30);
            /* Check if data line is still high after 30 us */
            if(HW_GPIO_READ(gpio)) {
                /* Read bit as 1 (MSB first) */
                result |= (1<<(7-i));
            }
            /* Reset timeout counter */
            cnt = 0;
            /* Wait for the data line to become low */
            while(HW_GPIO_READ(gpio)) {
                cnt++;
                /* Check if timeout has been reached */
                if(cnt > DHT_TIMING_TIMEOUT) {
                    /* Re-enable interrupts */
                    sei();
                    /* Timeout reached */
                    return DHT_RET_ERROR_TIMEOUT;
                }
            }
        }
        /* Store received byte in device data structure */
        dev->data[j] = result;
    }

    /*** Timing-critical section finished ***/
    /* Restore interrupts */
    sei();
    
    /* Reset the data line */
    HW_GPIO_OUTPUT(gpio);
    HW_GPIO_HIGH(gpio);
    
    /* Check if the received CRC matches the data */
    if(dev->data[4] == ((dev->data[0] + dev->data[1] + dev->data[2] + dev->data[3]) & 0xFF)) {
        /* Reading was successful */
        return DHT_RET_OK;
    } else {
        /* Reading failed */
#if DHT_CHECK_LAST_MEAS
        lasttime = 0;
#endif
        return DHT_RET_ERROR_CRC;
    }
}


/*!
 * Read the temperature in degree Celsius (°C) from a DHT sensor.
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  temperature     Temperature reading in degree Celsius (°C)
 * @return      OK* in case of success; ERROR* otherwise
 */
DHT_RET_t dht_get_temperature(DHT_t* dev, float* temperature) {
    float tmp;
    /* Call reading function */
    return dht_get_temperature_humidity(dev, temperature, &tmp);
}


/*!
 * Read the relative humidity (%RH) from a DHT sensor.
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  humidity        Relative humidity reading (%RH)
 * @return      OK* in case of success; ERROR* otherwise
 */
DHT_RET_t dht_get_humidity(DHT_t* dev, float* humidity) {
    float tmp;
    /* Call reading function */
    return dht_get_temperature_humidity(dev, &tmp, humidity);
}


/*!
 * Read both temperature and humidity from a DHT sensor.
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  temperature     Temperature reading in degree Celsius (°C)
 * @param[out]  humidity        Relative humidity reading (%RH)
 * @return      OK* in case of success; ERROR* otherwise
 */
DHT_RET_t dht_get_temperature_humidity(DHT_t* dev, float* temperature, float* humidity) {
    float tmp1,tmp2;
    /* Perform a low level read from the sensor */
    DHT_RET_t ret = _read(dev);
    if(ret >= DHT_RET_OK) {
        /* Reading temperature format depends on sensor type */
        switch(dev->type) {
            case DHT_DEV_DHT11:
                tmp1 = dev->data[2];
                if (dev->data[3] & 0x80) {
                    tmp1 = -1 - tmp1;
                }
                tmp1 += (dev->data[3] & 0x0F) * 0.1;
                break;
            case DHT_DEV_DHT12:
                tmp1 = dev->data[2];
                tmp1 += (dev->data[3] & 0x0F) * 0.1;
                if (dev->data[2] & 0x80) {
                    tmp1 *= -1;
                }
                break;
            case DHT_DEV_DHT21:
            case DHT_DEV_DHT22:
                tmp1 = ((int16_t)(dev->data[2] & 0x7F) << 8) | dev->data[3];
                tmp1 *= 0.1;
                if (dev->data[2] & 0x80) {
                    tmp1 *= -1;
                }
                break;
            default:
                /* Unsupported/unknown sensor device */
                return DHT_RET_ERROR_TYPE;
        }
        /* Reading humidity format depends on sensor type */
        switch(dev->type) {
            case DHT_DEV_DHT11:
            case DHT_DEV_DHT12:
                tmp2 = dev->data[0] + dev->data[1] * 0.1;
                break;
            case DHT_DEV_DHT21:
            case DHT_DEV_DHT22:
                tmp2 = ((int16_t)(dev->data[0]) << 8) | dev->data[1];
                tmp2 *= 0.1;
                break;
            default:
                /* Unsupported/unknown sensor device */
                return DHT_RET_ERROR_TYPE;
        }
    } else {
        /* Reset sensor */
        dht_reset(dev);
        /* Reading the sensor failed */
        return ret;
    }
    /* Return the acquired temperature and humidity with success */
    *temperature = tmp1;
    *humidity = tmp2;
    /* Read was successful */
    return DHT_RET_OK;
}
