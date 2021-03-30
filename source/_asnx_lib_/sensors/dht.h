/**
 *  Header file for DHT sensor functionality.
 * 
 *  Adapted taken from Arduino:
 *  -> https://github.com/adafruit/DHT-sensor-library
 *  -> https://github.com/adafruit/Adafruit_Sensor
 *  In combination with:
 *  -> https://www.electroschematics.com/arduino-dht22-am2302-tutorial-library/
 */

#ifndef _SEN_DHT_H_
#define _SEN_DHT_H_

/***** INCLUDES ***************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "owi/onewire.h"


/***** MACROS *****************************************************************/
/*** Timing-related ***/
#define DHT_TIMING_MAX                  (85)
#define DHT_TIMING_ROUND                (255)
#define DHT_TIMING_TIMEOUT_1000         (16000UL)
#define DHT_TIMING_MIN_INTERVAL         (2200UL)
/*** Device types ***/
#define DHT_DEV_NA                      (0)
#define DHT_DEV_DHT11                   (11)
#define DHT_DEV_DHT12                   (12)
#define DHT_DEV_DHT21                   (21)
#define DHT_DEV_DHT22                   (22)
#define DHT_DEV_AM2301                  (DHT_DEV_DHT21)
#define DHT_DEV_AM2302                  (DHT_DEV_DHT22)
/*** Function return values ***/
#define DHT_READ_FAIL                   (0)
#define DHT_READ_SUCCESS                (1)
#define DHT_READ_LAST_MEASUREMENT       (2)
#define DHT_READ_TIMEOUT                (0xFFFFFFFF)
#define DHT_READ_NAN                    (999.9)
/*** Cycle Count Threshold ***/
#define DHT_CCT_USED                    (DHT_CCT_AVR)
#define DHT_CCT_AVR                     (6)
#define DHT_CCT_DUE                     (30)
/*** Helper Makros ***/
#define DHT_STATE_HIGH                  (1)
#define DHT_STATE_LOW                   (0)


/***** GLOBAL VARIABLES *******************************************************/


/***** ENUMERATION ************************************************************/


/***** STRUCTURES *************************************************************/
typedef struct {
    onewire_t gpio;
    uint8_t type;
    uint8_t firstread;
    uint8_t data[5];
} dht_t;


/***** FUNCTION PROTOTYPES ****************************************************/
void dht_init(dht_t* dev, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin, uint8_t type);
float dht_get_temperature(dht_t* dev);
float dht_get_humidity(dht_t* dev);


/***** INLINE FUNCTIONS *******************************************************/


#endif // _SEN_DHT_H_
