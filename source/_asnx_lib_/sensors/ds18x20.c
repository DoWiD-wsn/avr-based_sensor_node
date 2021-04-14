/*****
 * @brief   ASN(x) DS18X20 temperature sensor library
 *
 * Library to support the DS18X20 temperature sensor.
 *
 * @file    /_asnx_lib_/sensors/ds18x20.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/14 $
 * @see     https://create.arduino.cc/projecthub/TheGadgetBoy/ds18b20-digital-temperature-sensor-and-arduino-9cc806
 *****/


/***** INCLUDES ***************************************************************/
#include "ds18x20.h"
/*** AVR ***/
#include <util/delay.h>


/***** LOCAL FUNCTION PROTOTYPES **********************************************/
int16_t _get_raw(ds18x20_t* dev);
float _raw2celsius(int16_t raw);


/***** FUNCTIONS **************************************************************/
/***
 * Initialization of a DS18x20 sensor instance.
 * 
 * @param[out]  dev     Pointer to the device structure to be filled
 * @param[in]   ddr     Pointer to the GPIO's DDRx register
 * @param[in]   port    Pointer to the GPIO's PORTx register
 * @param[in]   pin     Pointer to the GPIO's PINx register
 * @param[in]   portpin Index of the GPIO pin
 ***/
void ds18x20_init(ds18x20_t* dev, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin) {
    uint8_t i;
    /* Fill the HW GPIO structure */
    dev->gpio.ddr = ddr;
    dev->gpio.port = port;
    dev->gpio.pin = pin;
    dev->gpio.portpin = portpin;
    /* Initially, fill address space with zeros */
    for(i=0; i<8; i++) {
        dev->addr[i] = 0;
    }
    /* Initially, device type is not known */
    dev->type = DS18X20_DEV_NA;
}


/***
 * Find a DS18x20 sensor on the OWI line.
 * 
 * @param[in]   dev     Pointer to the device structure to be filled
 * @return      0 if device was found; !=0 otherwise (see DS18X20_RET_t)
 ***/
DS18X20_RET_t ds18x20_find(ds18x20_t* dev) {
    /* Search for a onewire device */
    if(!owi_search(&(dev->gpio),dev->addr)) {
        /* Reset search state */
        owi_search_reset();
        /* Return no device found */
        return DS18X20_RET_NO_DEV;
    }
    /* Check the retrieved CRC value */
    if(owi_crc8(dev->addr,7) != dev->addr[7]) {
        /* Return wrong CRC */
        return DS18X20_RET_WRONG_CRC;
    }
    /* The first ROM byte indicates sensor type */
    switch(dev->addr[0]) {
        case 0x10:      /**< DS18S20 (or old DS1820) */
            dev->type = DS18X20_DEV_DS18S20;
            break;
        case 0x28:      /**< DS18B20 */
            dev->type = DS18X20_DEV_DS18B20;
            break;
        case 0x22:      /**< DS1822 */
            dev->type = DS18X20_DEV_DS1822;
            break;
        default:
            /* Reset search state */
            owi_search_reset();
            /* Return wrong sensor type */
            return DS18X20_RET_WRONG_DEV;
    }
    return DS18X20_RET_OK;
}


/***
 * Read the raw temperature from a DS18x20 sensor.
 * 
 * @param[in]   dev     Pointer to the device structure to be filled
 * @return      Raw temperature reading
 ***/
int16_t _get_raw(ds18x20_t* dev) {
    uint8_t i;
    /* Temporary results */
    uint8_t data[9]={0};
    int16_t raw=0;
    
    /* Reset the onewire bus */
    owi_reset(&(dev->gpio));
    /* Check if a device has been found */
    if(dev->type == DS18X20_DEV_NA) {
        /* No device found */
        return DS18X20_RET_CANNOT_READ;
    }
    /* Select the previously found device */
    owi_select(&(dev->gpio),dev->addr);
    /* Start conversion, with parasite power on at the end */
    owi_write_byte(&(dev->gpio), DS18X20_COM_CONVERT, OWI_PARASITE_ON);

    /* Give the sensor some time to get the result */
    _delay_ms(DS18X20_CONV_DELAY);
    
    /* We might do a depower here, but the reset will take care of it */
    (void)owi_reset(&(dev->gpio));
    /* Select the device again */
    owi_select(&(dev->gpio),dev->addr);
    /* Request the temperature value */
    owi_write_byte(&(dev->gpio), DS18X20_COM_READ, OWI_PARASITE_ON);
    /* Read in nine data bytes */
    for(i=0; i<9; i++) {
        data[i] = owi_read_byte(&(dev->gpio));
    }
    
    /* Convert the (raw) data to actual temperature */
    raw = (data[1] << 8) | data[0];
    /* The further processing depends on the sensor type */
    if(dev->type == DS18X20_DEV_DS18S20) {
        /* 9 bit resolution default */
        raw = raw << 3;
        /* Check the result resolution */
        if(data[7] == 0x10) {
          /* "count remain" gives full 12 bit resolution */
          raw = (raw & 0xFFF0) + 12 - data[6];
        }
    } else {
        uint8_t cfg = (data[4] & 0x60);
        /* At lower res, the low bits are undefined, so let's zero them */
        if (cfg == DS18X20_CONF_9BIT) {
            raw = raw & ~7;
        } else if (cfg == DS18X20_CONF_10BIT) {
            raw = raw & ~3;
        } else if (cfg == DS18X20_CONF_11BIT) {
            raw = raw & ~1;
        }
    }
    /* Return the raw temperature value */
    return raw;
}


/***
 * Convert the raw temperature reading to degree Celsius (°C).
 * 
 * @param[in]   raw     Raw temperature reading
 * @return      Temperature in degree celsius (°C)
 ***/
float _raw2celsius(int16_t raw) {
    /* Calculation, see DS18x20 datasheet */
    return ((float)raw/16.0);
}


/***
 * Read the corresponding temperature in degree Celsius (°C) from a DS18x20 sensor.
 * 
 * @param[in]   dev     Pointer to the device structure to be filled
 * @return      Temperature reading in degree Celsius (°C)
 ***/
float ds18x20_get_temperature(ds18x20_t* dev) {
    /* Return the converted raw reading */
    return _raw2celsius(_get_raw(dev));
}
