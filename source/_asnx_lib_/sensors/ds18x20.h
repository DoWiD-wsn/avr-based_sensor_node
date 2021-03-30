/**
 *  Header file for DS18x20 sensor.
 * 
 *  https://create.arduino.cc/projecthub/TheGadgetBoy/ds18b20-digital-temperature-sensor-and-arduino-9cc806
 */

#ifndef _SENS_DS18X20_H_
#define _SENS_DS18X20_H_

/***** INCLUDES ***************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "owi/onewire.h"


/***** MACROS *****************************************************************/
/*** Conversion delay ***/
#define DS18x20_CONV_DELAY              (1000)        /* Maybe 750ms is enough*/

/*** Return values ***/
#define DS18X20_RET_OK                  (0)
#define DS18X20_RET_FAIL                (1)
#define DS18X20_RET_NO_DEV              (10)
#define DS18X20_RET_WRONG_DEV           (11)
#define DS18X20_RET_WRONG_CRC           (12)
#define DS18X20_RET_CANNOT_READ         (-390)

/*** Device types ***/
#define DS18X20_DEV_NA                  (0)
#define DS18X20_DEV_DS18S20             (1)
#define DS18X20_DEV_DS18B20             (2)
#define DS18X20_DEV_DS1822              (3)

/*** DS18x20 commands ***/
/* ROM */
#define DS18X20_COM_ROM_SEARCH          (0xF0)
#define DS18X20_COM_ROM_READ            (0x33)
#define DS18X20_COM_ROM_MATCH           (0x55)
#define DS18X20_COM_ROM_SKIP            (0xCC)
#define DS18X20_COM_ROM_ALARM_SEARCH    (0xEC)
/* Function */
#define DS18X20_COM_CONVERT             (0x44)
#define DS18X20_COM_READ                (0xBE)
#define DS18X20_COM_WRITE               (0x4E)
#define DS18X20_COM_COPY                (0x48)
#define DS18X20_COM_RECALL              (0xB8)
#define DS18X20_COM_READ_SUPPLY         (0xB4)

/*** DS18x20 configuration ***/
#define DS18X20_CONF_9BIT               (0x00)
#define DS18X20_CONF_10BIT              (0x20)
#define DS18X20_CONF_11BIT              (0x40)
#define DS18X20_CONF_12BIT              (0x60)


/***** GLOBAL VARIABLES *******************************************************/


/***** ENUMERATION ************************************************************/


/***** STRUCTURES *************************************************************/
typedef struct {
    onewire_t gpio;
    uint8_t addr[8];
    uint8_t type;
} ds18x20_t;


/***** FUNCTION PROTOTYPES ****************************************************/
void ds18x20_init(ds18x20_t* dev, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin);
uint8_t ds18x20_find(ds18x20_t* dev);
int16_t ds18x20_get_temperature(ds18x20_t* dev);
float ds18x20_raw2celsius(int16_t raw);


/***** INLINE FUNCTIONS *******************************************************/


#endif // _SENS_DS18X20_H_
