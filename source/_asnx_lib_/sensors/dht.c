/*****
 * @brief   ASN(x) DHT temperature/humidity sensor library
 *
 * Library to support the DHT temperature/humidity sensor.
 *
 * @file    /_asnx_lib_/sensors/dht.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/14 $
 * @see     https://github.com/adafruit/DHT-sensor-library
 * @see     https://www.electroschematics.com/arduino-dht22-am2302-tutorial-library/
 *****/


/***** INCLUDES *******************************************************/
#include "dht.h"
/*** AVR ***/
#include <avr/interrupt.h>
#include <util/delay.h>
/*** ASNX LIB ***/
#include "timer/systick.h"


/***** LOCAL FUNCTION PROTOTYPES **************************************/
uint8_t _read(dht_t* dev);
uint32_t _expect_pulse(dht_t* dev, uint8_t level);


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
void dht_init(dht_t* dev, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin, DHT_DEV_t type) {
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
    HW_GPIO_INPUT(gpio);
    HW_GPIO_LOW(gpio);
    /* Power-up delay (1s) */
    _delay_ms(DHT_TIMING_STARTUP);
}


/***
 * Perform a raw reading from the sensor.
 * 
 * @param[in]   dev     Pointer to the device structure to be filled
 * @return      SUCCESS in case of success; FAIL, LAST_MEASUREMENT, or TIMEOUT otherwise
 ***/
uint8_t _read(dht_t* dev) {
    /* Get a pointer to the HW GPIO structure */
    hw_io_t* gpio = &(dev->gpio);
    /* Temporary variables */
    uint32_t cycles[80];
    uint8_t i;
    
    /*** Check if sensor was read less than two seconds ago ***/
    static uint32_t lasttime = 0;
    /* Read in the current tick count (ms) */
    uint32_t currenttime = systick_get_ticks();
    /* Check if there was already a previous reading */
    if(lasttime != 0) {
        /* Check if at least 2 seconds (2000ms) have elapsed) */
        if((currenttime - lasttime) < DHT_TIMING_MIN_INTERVAL) {
            /* Too less time, use old measurement result */
            return DHT_READ_LAST_MEASUREMENT;
        }
    }
    /* Update last reading time */
    lasttime = currenttime;
    
    /* Reset result data section */
    for(i=0; i<5; i++) {
        dev->data[i] = 0;
    }

    /*** Send start signal ***/
    /* Release bus */
    HW_GPIO_INPUT(gpio);
    /* Wait for 1ms */
    _delay_ms(1);

    /* First set data line low ... */
    HW_GPIO_OUTPUT(gpio);
    HW_GPIO_LOW(gpio);
    /* ... for a period according to sensor type */
    switch(dev->type) {
        case DHT_DEV_DHT21:
        case DHT_DEV_DHT22:
            /* Datasheet says "at least 1ms", 2ms just to be safe */
            _delay_ms(2);
            break;
        case DHT_DEV_DHT11:
        case DHT_DEV_DHT12:
        default:
            /* Datasheet says "at least 18ms", 20ms just to be safe */
            _delay_ms(20);
            break;
    }

    /*** Start timing-critical section ***/
    /* Disable interrupts */
    cli();
    
    /* Release bus */
    HW_GPIO_INPUT(gpio);
    /* Wait for 40us */
    _delay_us(40);

    /* First expect a low signal for ~80us followed by a high signal for ~80us again */
    if(_expect_pulse(dev,HW_STATE_LOW) == DHT_READ_TIMEOUT) {
        return DHT_READ_FAIL;
    }
    if(_expect_pulse(dev,HW_STATE_HIGH) == DHT_READ_TIMEOUT) {
        return DHT_READ_FAIL;
    }

    /* Now read the 40 bits sent by the sensor.
     * Each bit is sent as a 50ms LOW pulse followed by a variable length
     * HIGH pulse. If the HIGH pulse is ~28ms then it's a "0" and if it's
     * ~70ms then it's a "1". We measure the cycle count of the initial
     * 50us low pulse and use that to compare to the cycle count of the
     * HIGH pulse to determine if the bit is a
     * -> 0 (HIGH state cycle count < low state cycle count), or a
     * -> 1 (HIGH state cycle count > low state cycle count).
     * Note that for speed all the pulses are read into a array and then
     * examined in a later step. */
    for(i=0; i<80; i+=2) {
        cycles[i] = _expect_pulse(dev,HW_STATE_LOW);
        cycles[i+1] = _expect_pulse(dev,HW_STATE_HIGH);
    }
    /*** Timing-critical section finished ***/
    /* Restore interrupts */
    sei();

    /* Inspect pulses and determine which ones are 0 or 1 */
    for(i=0; i<40; ++i) {
        uint32_t low_cycles = cycles[2*i];
        uint32_t high_cycles = cycles[2*i + 1];
        /* Check if at least one pulse was out of time */
        if ((low_cycles == DHT_READ_TIMEOUT) || (high_cycles == DHT_READ_TIMEOUT)) {
            return DHT_READ_FAIL;
        }
        /* Shift a "1" to the current position */
        dev->data[i/8] <<= 1;
        /* Now compare the low and high cycle times to see if the bit is a "0" or "1". */
        if(high_cycles > low_cycles) {
          /* High cycles are greater than 50us low cycle count, must be a "1". */
          dev->data[i/8] |= 1;
        }
        /* Else high cycles are less than (or equal to, a weird case) the 50us low
         * cycle count so this must be a zero.  Nothing needs to be changed in the
         * stored data. */
    }

    /* Check if we read 40 bits and that the checksum matches */
    if(dev->data[4] == ((dev->data[0] + dev->data[1] + dev->data[2] + dev->data[3]) & 0xFF)) {
        /* Reading was successful */
        return DHT_READ_SUCCESS;
    } else {
        /* Reading failed */
        return DHT_READ_FAIL;
    }
}


/***
 * Wait for a pulse (response) from the sensor.
 * 
 * @param[in]   dev     Pointer to the device structure to be filled
 * @param[in]   level   Logic level of the expected pulse
 * @return      Number of cycles until the pulse arrived; TIMEOUT otherwise
 ***/
uint32_t _expect_pulse(dht_t* dev, uint8_t level) {
    /* Get a pointer to the HW GPIO structure */
    hw_io_t* gpio = &(dev->gpio);
    /* Temporary variables */
    uint32_t count = 0;
    /* Wait for a level change */
    while((HW_GPIO_READ(gpio) ? HW_STATE_HIGH : HW_STATE_LOW) == level) {
        /* Check if timeout was reached */
        if (count++ >= DHT_TIMING_TIMEOUT_1000) {
            /* // Exceeded timeout, fail */
            return DHT_READ_TIMEOUT;
        }
    }
    /* Return the count of cycles */
    return count;
}


/***
 * Read the temperature in degree Celsius (°C) from a DHT sensor.
 * 
 * @param[in]   dev     Pointer to the device structure to be filled
 * @return      Temperature reading in degree Celsius (°C)
 ***/
float dht_get_temperature(dht_t* dev) {
    /* Intermediate temperature value */
    float temp = DHT_READ_NAN;
    int16_t inter = 0;
    /* Read current value from the sensor */
    if(_read(dev) >= DHT_READ_SUCCESS) {
        /* Reading format depends on sensor type */
        switch(dev->type) {
            case DHT_DEV_DHT11:
                temp = dev->data[2];
                if (dev->data[3] & 0x80) {
                    temp = -1 - temp;
                }
                temp += (dev->data[3] & 0x0F);
                temp /= 10.0;
                break;
            case DHT_DEV_DHT12:
                temp = dev->data[2];
                temp += (dev->data[3] & 0x0F);
                temp /= 10.0;
                if (dev->data[2] & 0x80) {
                    temp *= -1;
                }
                break;
            case DHT_DEV_DHT21:
            case DHT_DEV_DHT22:
                inter = ((int16_t)(dev->data[2] & 0x7F) << 8) | (int16_t)(dev->data[3]);
                temp = (float)(inter) / 10.0;
                if (dev->data[2] & 0x80) {
                    temp *= -1;
                }
                break;
            default:
                /* Unsupported/unknown sensor device */
                temp = DHT_READ_NAN;
                break;
        }
    } else {
        /* Reading the sensor failed */
        temp = DHT_READ_NAN;
    }
    /* Return the acquired temperature (or NAN) */
    return temp;
}


/***
 * Read the relative humidity (%RH) from a DHT sensor.
 * 
 * @param[in]   dev     Pointer to the device structure to be filled
 * @return      Relative humidity reading (%RH)
 ***/
float dht_get_humidity(dht_t* dev) {
    /* Intermediate humidity value */
    float temp = DHT_READ_NAN;
    int16_t inter = 0;
    /* Read current value from the sensor */
    if(_read(dev) >= DHT_READ_SUCCESS) {
        /* Reading format depends on sensor type */
        switch(dev->type) {
            case DHT_DEV_DHT11:
            case DHT_DEV_DHT12:
                temp = dev->data[0] + dev->data[1] * 0.1;
                break;
            case DHT_DEV_DHT21:
            case DHT_DEV_DHT22:
                inter = ((int16_t)(dev->data[0]) << 8) | (int16_t)(dev->data[1]);
                temp = (float)(inter) / 10.0;
                break;
            default:
                /* Unsupported/unknown sensor device */
                temp = DHT_READ_NAN;
                break;
        }
    } else {
        /* Reading the sensor failed */
        temp = DHT_READ_NAN;
    }
    /* Return the acquired humidity (or NAN) */
    return temp;
}
