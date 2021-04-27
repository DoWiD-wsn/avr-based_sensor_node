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
static DS18X20_RET_t _find(DS18X20_t* dev);
static DS18X20_RET_t _get_raw(DS18X20_t* dev, int16_t* raw);
static float _raw2celsius(int16_t raw);


/***** FUNCTIONS **************************************************************/
/***
 * Initialization of a DS18x20 sensor instance.
 * 
 * @param[out]  dev     Pointer to the device structure to be filled
 * @param[in]   ddr     Pointer to the GPIO's DDRx register
 * @param[in]   port    Pointer to the GPIO's PORTx register
 * @param[in]   pin     Pointer to the GPIO's PINx register
 * @param[in]   portpin Index of the GPIO pin
 * @return      OK in case of success; ERROR otherwise
 ***/
DS18X20_RET_t ds18x20_init(DS18X20_t* dev, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin) {
    /* Fill the HW GPIO and data structure */
    owi_get(&(dev->gpio), &(dev->data), ddr, port, pin, portpin);
    /* Initially, device type is not known */
    dev->type = DS18X20_DEV_NA;
    /* Try to find a sensor on the OWI data line */
    return _find(dev);
}


/***
 * Find a DS18x20 sensor on the OWI line.
 * 
 * @param[in]   dev     Pointer to the device structure to be filled
 * @return      OK in case of success; ERROR* otherwise
 ***/
DS18X20_RET_t _find(DS18X20_t* dev) {
    /* Search for a onewire device */
    if(!owi_search(&(dev->gpio),&(dev->data),dev->addr)) {
        /* Reset search state */
        owi_search_reset(&(dev->data));
        /* Return no device found */
        return DS18X20_RET_ERROR_NODEV;
    }
    /* Check the retrieved CRC value */
    if(owi_crc8(dev->addr,7) != dev->addr[7]) {
        /* Return wrong CRC */
        return DS18X20_RET_ERROR_CRC;
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
            owi_search_reset(&(dev->data));
            /* Return wrong sensor type */
            return DS18X20_RET_ERROR_DEV;
    }
    return DS18X20_RET_OK;
}


/***
 * Read the raw temperature from a DS18x20 sensor.
 * 
 * @param[in]   dev     Pointer to the device structure to be filled
 * @param[out]  raw     Raw temperature reading
 * @return      OK in case of success; ERROR* otherwise
 ***/
static DS18X20_RET_t _get_raw(DS18X20_t* dev, int16_t* raw) {
    uint8_t i;
    /* Temporary results */
    uint8_t data[9]={0};
    int16_t tmp=0;
    
    /* Reset the onewire bus */
    owi_reset(&(dev->gpio));
    /* Check if a device has been found */
    if(dev->type == DS18X20_DEV_NA) {
        /* No device found */
        return DS18X20_RET_ERROR_READ;
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
    tmp = (data[1] << 8) | data[0];
    /* The further processing depends on the sensor type */
    if(dev->type == DS18X20_DEV_DS18S20) {
        /* 9 bit resolution default */
        tmp = tmp << 3;
        /* Check the result resolution */
        if(data[7] == 0x10) {
          /* "count remain" gives full 12 bit resolution */
          tmp = (tmp & 0xFFF0) + 12 - data[6];
        }
    } else {
        uint8_t cfg = (data[4] & 0x60);
        /* At lower res, the low bits are undefined, so let's zero them */
        if (cfg == DS18X20_CONF_9BIT) {
            tmp = tmp & ~7;
        } else if (cfg == DS18X20_CONF_10BIT) {
            tmp = tmp & ~3;
        } else if (cfg == DS18X20_CONF_11BIT) {
            tmp = tmp & ~1;
        }
    }
    /* Return the raw temperature value */
    *raw = tmp;
    return DS18X20_RET_OK;
}


/***
 * Convert the raw temperature reading to degree Celsius (°C).
 * 
 * @param[in]   raw     Raw temperature reading
 * @return      Temperature in degree celsius (°C)
 ***/
static float _raw2celsius(int16_t raw) {
    /* Calculation, see DS18x20 datasheet */
    return ((float)raw/16.0);
}


/***
 * Read the corresponding temperature in degree Celsius (°C) from a DS18x20 sensor.
 * 
 * @param[in]   dev     Pointer to the device structure to be filled
 * @return      Temperature reading in degree Celsius (°C)
 ***/
DS18X20_RET_t ds18x20_get_temperature(DS18X20_t* dev, float* temperature) {
    int16_t raw;
    DS18X20_RET_t ret = _get_raw(dev, &raw);
    if(ret == DS18X20_RET_OK) {
        *temperature = _raw2celsius(raw);
        return DS18X20_RET_OK;
    } else {
        return ret;
    }
}
