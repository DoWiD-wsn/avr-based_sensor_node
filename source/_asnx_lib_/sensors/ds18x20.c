/**
 *  Source file for DS18x20 sensor.
 */

/***** INCLUDES ***************************************************************/
#include "sensors/ds18x20.h"
#include <util/delay.h>


/***** GLOBAL VARIABLES *******************************************************/


/***** LOCAL FUNCTION PROTOTYPES **********************************************/


/***** WRAPPER FUNCTIONS ******************************************************/


/***** FUNCTIONS **************************************************************/
/*
 * Initialization of the DS18B20 sensor
 * 
 * ds18x20_t* dev       Pointer to the structure to be filled
 * uint8_t ddr          DDRx
 * uint8_t port         PORTx
 * uint8_t pin          PINx
 * uint8_t portpin      Portpin, e.g., PD2
 */
void ds18x20_init(ds18x20_t* dev, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin) {
    uint8_t i;
    dev->gpio.ddr = ddr;
    dev->gpio.port = port;
    dev->gpio.pin = pin;
    dev->gpio.portpin = portpin;
    for(i=0; i<8; i++) {
        dev->addr[i] = 0;
    }
    dev->type = DS18X20_DEV_NA;
}


/*
 * Find an DS18x20 sensor
 * 
 * ds18x20_t* dev       Pointer to the device structure
 * return               1 if sensor was found; 0 otherwise
 */
uint8_t ds18x20_find(ds18x20_t* dev) {
    /* Search for a onewire device */
    if(!onewire_search(&(dev->gpio),dev->addr)) {
        /* Reset search state */
        onewire_search_reset();
        /* Return no device found */
        return DS18X20_RET_NO_DEV;
    }
    /* Check the retrieved CRC value */
    if(onewire_crc8(dev->addr,7) != dev->addr[7]) {
        /* Return wrong CRC */
        return DS18X20_RET_WRONG_CRC;
    }
    /* The first ROM byte indicates sensor type */
    switch(dev->addr[0]) {
        case 0x10:      // DS18S20 (or old DS1820)
                        // s=1
            dev->type = DS18X20_DEV_DS18S20;
            break;
        case 0x28:      // DS18B20
                        // s=0
            dev->type = DS18X20_DEV_DS18B20;
            break;
        case 0x22:      // DS1822
                        // s=0
            dev->type = DS18X20_DEV_DS1822;
            break;
        default:
            /* Reset search state */
            onewire_search_reset();
            /* Return wrong sensor type */
            return DS18X20_RET_WRONG_DEV;
    }
    return DS18X20_RET_OK;
}


/*
 * Read temperature from an DS18x20 sensor
 * 
 * ds18x20_t* dev       Pointer to the device structure
 * return               Raw temperature reading
 */
int16_t ds18x20_get_temperature(ds18x20_t* dev) {
    uint8_t i;
    /* Temporary results */
    uint8_t data[9]={0};
    int16_t raw=0;
    
    /* Reset the onewire bus */
    onewire_reset(&(dev->gpio));
    /* Check if a device has been found */
    if(dev->type == DS18X20_DEV_NA) {
        /* No device found */
        return DS18X20_RET_CANNOT_READ;
    }
    /* Select the previously found device */
    onewire_select(&(dev->gpio),dev->addr);
    /* Start conversion, with parasite power on at the end */
    onewire_write_byte(&(dev->gpio), DS18X20_COM_CONVERT, ONEWIRE_PARASITE_ON);

    /* Give the sensor some time to get the result */
    _delay_ms(DS18x20_CONV_DELAY);
    
    /* We might do a depower here, but the reset will take care of it */
    (void)onewire_reset(&(dev->gpio));
    /* Select the device again */
    onewire_select(&(dev->gpio),dev->addr);
    /* Request the temperature value */
    onewire_write_byte(&(dev->gpio), DS18X20_COM_READ, ONEWIRE_PARASITE_ON);
    /* Read in nine data bytes */
    for(i=0; i<9; i++) {
        data[i] = onewire_read_byte(&(dev->gpio));
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


/*
 * Convert the raw temperature reading to degree celsius
 * 
 * int16_t raw          Raw temperature reading
 * return               Temperature in degree celsius
 */
float ds18x20_raw2celsius(int16_t raw) {
    /* Calculation, see datasheet */
    return ((float)raw/16.0);
}
