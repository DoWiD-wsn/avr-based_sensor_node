/*****
 * @brief   ASN(x) DHT temperature/humidity sensor library
 *
 * Library to support the DHT temperature/humidity sensor.
 * The low level reading is partly based on the code of Davide Gironi.
 *
 * @file    /_asnx_lib_/sensors/dht.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1.1 $
 * @date    $Date: 2021/05/18 $
 * @see     http://davidegironi.blogspot.com/2013/02/reading-temperature-and-humidity-on-avr.html
 *****/

#ifndef _ASNX_DHT_H_
#define _ASNX_DHT_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdio.h>
#include <stdint.h>
/*** ASNX LIB ***/
#include "hw/hw.h"


/***** DEFINES ********************************************************/
/* Enable last-measurement time check (requires systick lib) */
#define DHT_CHECK_LAST_MEAS     (1)

/* Timing-related */
#define DHT_TIMING_MIN_INTERVAL 2000UL
#define DHT_TIMING_TIMEOUT      200


/***** ENUMERATION ****************************************************/
/* Enumeration for the DHT function return values */
typedef enum {
    DHT_RET_ERROR_NOREADING     = -5,
    DHT_RET_ERROR_TYPE          = -4,
    DHT_RET_ERROR_CRC           = -3,
    DHT_RET_ERROR_TIMEOUT       = -2,
    DHT_RET_ERROR_START         = -1,
    DHT_RET_OK                  = 0,
    DHT_RET_OK_LAST_VALUE       = 1
} DHT_RET_t;

/* Enumeration for the DHT device types */
typedef enum {
    DHT_DEV_NA                  = 0,
    DHT_DEV_DHT11               = 11,
    DHT_DEV_DHT12               = 12,
    DHT_DEV_DHT21               = 21,
    DHT_DEV_DHT22               = 22
} DHT_DEV_t;
/* AM230x sensor use the DHT sensors */
#define DHT_DEV_AM2301          (DHT_DEV_DHT21)
#define DHT_DEV_AM2302          (DHT_DEV_DHT22)


/***** STRUCTURES *****************************************************/
/***
 * A structure to store the DHT module properties.
 ***/
typedef struct {
    hw_io_t gpio;       /**< OWI GPIO handle */
    uint8_t type;       /**< Sensor type */
    uint8_t data[5];    /**< Last data read from the sensor */
    uint8_t valid;      /**< Flag whether a valid read was performed */
} DHT_t;


/***** FUNCTION PROTOTYPES ********************************************/
void dht_init(DHT_t* dev, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin, DHT_DEV_t type);
DHT_RET_t dht_read(DHT_t* dev);
DHT_RET_t dht_get_temperature(DHT_t* dev, float* temperature);
DHT_RET_t dht_get_humidity(DHT_t* dev, float* humidity);
DHT_RET_t dht_get_temperature_humidity(DHT_t* dev, float* temperature, float* humidity);


#endif // _ASNX_DHT_H_
