/*****
 * @brief   ASN(x) DHT temperature/humidity sensor library
 *
 * Library to support the DHT temperature/humidity sensor.
 * The low level reading is partly based on the code of Davide Gironi.
 *
 * @file    /_asnx_lib_/sensors/dht.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/14 $
 * @see     http://davidegironi.blogspot.com/2013/02/reading-temperature-and-humidity-on-avr.html
 *****/


/***** INCLUDES *******************************************************/
#include "dht.h"
/*** AVR ***/
#include <avr/interrupt.h>
#include <util/delay.h>
/*** ASNX LIB ***/
#include "timer/systick.h"


#include "uart/uart.h"
#include "util/printf.h"


/***** LOCAL FUNCTION PROTOTYPES **************************************/
static DHT_RET_t _read(DHT_t* dev);


/***** FUNCTIONS **************************************************************/
/***
 * Initialization of a DHT sensor instance.
 * 
 * @param[out]  dev     Pointer to the device structure to be filled
 * @param[in]   ddr     Pointer to the GPIO's DDRx register
 * @param[in]   port    Pointer to the GPIO's PORTx register
 * @param[in]   pin     Pointer to the GPIO's PINx register
 * @param[in]   portpin Index of the GPIO pin
 * @param[in]   type    DHT sensor type
 ***/
void dht_init(DHT_t* dev, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin, DHT_DEV_t type) {
    /* Fill the HW GPIO structure */
    dev->gpio.ddr = ddr;
    dev->gpio.port = port;
    dev->gpio.pin = pin;
    dev->gpio.portpin = portpin;
    /* Set the sensor type */
    dev->type = type;
    /* Get a pointer to the HW GPIO structure */
    hw_io_t* gpio = &(dev->gpio);
    /* Setup the hardware (pin) */
    HW_GPIO_OUTPUT(gpio);
    HW_GPIO_HIGH(gpio);
}


/***
 * Perform a low-level reading from the sensor.
 * 
 * @param[in]   dev     Pointer to the device structure
 * @return      OK* in case of success; ERROR* otherwise
 ***/
static DHT_RET_t _read(DHT_t* dev) {
    /* Get a pointer to the HW GPIO structure */
    hw_io_t* gpio = &(dev->gpio);
    /* Temporary counter variables */
    uint8_t i,j = 0;
    
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
            _delay_us(1100);        // TODO: davide uses 500 us
            break;
        case DHT_DEV_DHT11:
        case DHT_DEV_DHT12:
            /* Datasheet says "at least 18ms", 20ms just to be safe */
            _delay_ms(20);
            break;
        default:
            /* Unsupported sensor type */
            return DHT_RET_ERROR_TYPE;
    }
    /* Release the data line */
    HW_GPIO_HIGH(gpio);
    HW_GPIO_INPUT(gpio);
    _delay_us(40);
    
    /* Check start condition high level */
    if(HW_GPIO_READ(gpio)) {
        return DHT_RET_ERROR_START;
    }
    _delay_us(80);
    /* Check start condition low level */
    if(!HW_GPIO_READ(gpio)) {
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
    _delay_ms(100);
    
    /* Check if the received CRC matches the data */
    if(dev->data[4] == ((dev->data[0] + dev->data[1] + dev->data[2] + dev->data[3]) & 0xFF)) {
        /* Reading was successful */
        return DHT_RET_OK;
    } else {
        /* Reading failed */
        lasttime = 0;
        return DHT_RET_ERROR_CRC;
    }
}


/***
 * Read the temperature in degree Celsius (°C) from a DHT sensor.
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  temperature     Temperature reading in degree Celsius (°C)
 * @return      OK* in case of success; ERROR* otherwise
 ***/
DHT_RET_t dht_get_temperature(DHT_t* dev, float* temperature) {
    float tmp;
    /* Perform a low level read from the sensor */
    DHT_RET_t ret = _read(dev);
    if(ret >= DHT_RET_OK) {
        /* Reading format depends on sensor type */
        switch(dev->type) {
            case DHT_DEV_DHT11:
                tmp = dev->data[2];
                if (dev->data[3] & 0x80) {
                    tmp = -1 - tmp;
                }
                tmp += (dev->data[3] & 0x0F) * 0.1;
                break;
            case DHT_DEV_DHT12:
                tmp = dev->data[2];
                tmp += (dev->data[3] & 0x0F) * 0.1;
                if (dev->data[2] & 0x80) {
                    tmp *= -1;
                }
                break;
            case DHT_DEV_DHT21:
            case DHT_DEV_DHT22:
                tmp = ((int16_t)(dev->data[2] & 0x7F) << 8) | dev->data[3];
                tmp *= 0.1;
                if (dev->data[2] & 0x80) {
                    tmp *= -1;
                }
                break;
            default:
                /* Unsupported/unknown sensor device */
                return DHT_RET_ERROR_TYPE;
        }
    } else {
        /* Reading the sensor failed */
        return ret;
    }
    /* Return the acquired temperature with success */
    *temperature = tmp;
    return DHT_RET_OK;
}


/***
 * Read the relative humidity (%RH) from a DHT sensor.
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  humidity        Relative humidity reading (%RH)
 * @return      OK* in case of success; ERROR* otherwise
 ***/
DHT_RET_t dht_get_humidity(DHT_t* dev, float* humidity) {
    float tmp;
    /* Perform a low level read from the sensor */
    DHT_RET_t ret = _read(dev);
    if(ret >= DHT_RET_OK) {
        /* Reading format depends on sensor type */
        switch(dev->type) {
            case DHT_DEV_DHT11:
            case DHT_DEV_DHT12:
                tmp = dev->data[0] + dev->data[1] * 0.1;
                break;
            case DHT_DEV_DHT21:
            case DHT_DEV_DHT22:
                tmp = ((int16_t)(dev->data[0]) << 8) | dev->data[1];
                tmp *= 0.1;
                break;
            default:
                /* Unsupported/unknown sensor device */
                return DHT_RET_ERROR_TYPE;
        }
    } else {
        /* Reading the sensor failed */
        return ret;
    }
    /* Return the acquired humidity with success */
    *humidity = tmp;
    return DHT_RET_OK;
}


/***
 * Read both temperature and humidity from a DHT sensor.
 * 
 * @param[in]   dev             Pointer to the device structure
 * @param[out]  temperature     Temperature reading in degree Celsius (°C)
 * @param[out]  humidity        Relative humidity reading (%RH)
 * @return      OK* in case of success; ERROR* otherwise
 ***/
DHT_RET_t dht_get_temperature_humidity(DHT_t* dev, float* temperature, float* humidity) {
    DHT_RET_t ret;
    /* Read the temperature */
    ret = dht_get_temperature(dev, temperature);
    /* Check the return value */
    if(ret < DHT_RET_OK) {
        return ret;
    }
    /* Read the humidity */
    ret = dht_get_humidity(dev, humidity);
    /* Check the return value */
    if(ret < DHT_RET_OK) {
        return ret;
    }
    /* Read was successful */
    return DHT_RET_OK;
}
