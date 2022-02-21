/*!
 * @brief   ASN(x) DS18X20 temperature sensor library -- header file
 *
 * Library to support the DS18X20 temperature sensor.
 *
 * @file    /_asnx_lib_/sensors/ds18x20.h
 * @author  Dominik Widhalm
 * @version 1.2.1
 * @date    2021/12/29
 */

#ifndef _ASNX_DS18X20_H_
#define _ASNX_DS18X20_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
/*** AVR ***/
#include <util/delay.h>
/*** ASNX LIB ***/
#include "owi/owi.h"


/***** DEFINES ********************************************************/
/*! Conversion delay [ms] */
#define DS18X20_CONV_DELAY              (750)
/* ROM commands */
#define DS18X20_COM_ROM_SEARCH          (0xF0)
#define DS18X20_COM_ROM_READ            (0x33)
#define DS18X20_COM_ROM_MATCH           (0x55)
#define DS18X20_COM_ROM_SKIP            (0xCC)
#define DS18X20_COM_ROM_ALARM_SEARCH    (0xEC)
/* Function commands */
#define DS18X20_COM_CONVERT             (0x44)
#define DS18X20_COM_READ                (0xBE)
#define DS18X20_COM_WRITE               (0x4E)
#define DS18X20_COM_COPY                (0x48)
#define DS18X20_COM_RECALL              (0xB8)
#define DS18X20_COM_READ_SUPPLY         (0xB4)


/***** ENUMERATION ****************************************************/
/*! Enumeration for the DS18x20 function return values */
typedef enum {
    DS18X20_RET_ERROR_CRC       = -4,   /**< CRC check failed */
    DS18X20_RET_ERROR_DEV       = -3,   /**< Unsupported device */
    DS18X20_RET_ERROR_NODEV     = -2,   /**< No device found */
    DS18X20_RET_ERROR_READ      = -1,   /**< Reading failed */
    DS18X20_RET_OK              = 0     /**< Successful */
} DS18X20_RET_t;

/*! Enumeration for the DS18x20 device types */
typedef enum {
    DS18X20_DEV_NA              = 0,
    DS18X20_DEV_DS18S20         = 1,
    DS18X20_DEV_DS18B20         = 2,
    DS18X20_DEV_DS1822          = 3
} DS18X20_DEV_t;

/*! Enumeration for the DS18x20 resolution configuration */
typedef enum {
    DS18X20_CONF_9BIT           = 0x00,
    DS18X20_CONF_10BIT          = 0x20,
    DS18X20_CONF_11BIT          = 0x40,
    DS18X20_CONF_12BIT          = 0x60
} DS18X20_CONF_t;


/***** STRUCTURES *****************************************************/
/*!
 * A structure to store the DS18X20 module properties.
 */
typedef struct {
    hw_io_t gpio;       /**< OWI GPIO handle */
    uint8_t addr[8];    /**< Sensor address */
    DS18X20_DEV_t type; /**< Sensor type */
    OWI_DATA_t data;    /**< Sensor data structure */
} DS18X20_t;


/***** FUNCTION PROTOTYPES ********************************************/
DS18X20_RET_t ds18x20_init(DS18X20_t* dev, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin);
DS18X20_RET_t ds18x20_get_temperature(DS18X20_t* dev, float* temperature);


#endif // _ASNX_DS18X20_H_
