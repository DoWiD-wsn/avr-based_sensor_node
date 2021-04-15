/*****
 * @brief   ASN(x) DHT temperature/humidity sensor library
 *
 * Library to support the DHT temperature/humidity sensor.
 *
 * @file    /_asnx_lib_/sensors/dht.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/14 $
 * @see     https://github.com/adafruit/DHT-sensor-library
 * @see     https://www.electroschematics.com/arduino-dht22-am2302-tutorial-library/
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
/* Timing-related */
#define DHT_TIMING_STARTUP              (1000)
#define DHT_TIMING_MAX                  (85)
#define DHT_TIMING_ROUND                (255)
#define DHT_TIMING_TIMEOUT_1000         (16000UL)
#define DHT_TIMING_MIN_INTERVAL         (2200UL)
/* Function return values */
#define DHT_READ_FAIL                   (0)
#define DHT_READ_SUCCESS                (1)
#define DHT_READ_LAST_MEASUREMENT       (2)
#define DHT_READ_TIMEOUT                (0xFFFFFFFF)
#define DHT_READ_NAN                    (999.9)
/* Invalid result */
#define DHT_READ_NAN                    (999.9)

/***** ENUMERATION ****************************************************/
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
    uint8_t firstread;  /**< Flag to indicate the first reading */
    uint8_t data[5];    /**< Last data read from the sensor */
} DHT_t;


/***** FUNCTION PROTOTYPES ********************************************/
void dht_init(DHT_t* dev, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin, DHT_DEV_t type);
float dht_get_temperature(DHT_t* dev);
float dht_get_humidity(DHT_t* dev);


#endif // _ASNX_DHT_H_
