/*!
 * @brief   ASN(x) sensor network message library -- header file
 *
 * Library for functionality regarding the messaging of sensor values.
 *
 * @file    /_asnx_lib_/util/sensor_msg.h
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 */

#ifndef _ASNX_MSG_H_
#define _ASNX_MSG_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
/*** ASNX ***/
#include "util/fixed_point.h"


/***** DEFINES ********************************************************/
/*! Number of sensor measurements per message */
#ifndef SEN_MSG_NUM_MEASUREMENTS
/*! Default value is the maximum for the full demo (14 measurements) */
#  define SEN_MSG_NUM_MEASUREMENTS          (14)
#endif

/* Get SL & SH from 64-bit MAC */
#define SEN_MSG_MAC_SH(mac)                 ((uint32_t)((mac>>32) & 0xFFFFFFFF))
#define SEN_MSG_MAC_SL(mac)                 ((uint32_t)(mac & 0xFFFFFFFF))

/*!
 * Sensor node IDs (DoWiD's testbed)
 * @note    Update with your own Xbee MACs
 */
/* Full 64-bit MACs */
#define SEN_MSG_MAC_CH                      (0x0013A20041B9DCDC)
#define SEN_MSG_MAC_SN1                     (0x0013A20041B9F805)
#define SEN_MSG_MAC_SN2                     (0x0013A20041B9FFC2)
#define SEN_MSG_MAC_SN3                     (0x0013A20041B9FFD8)
#define SEN_MSG_MAC_SN4                     (0x0013A20041B9FFDD)
#define SEN_MSG_MAC_SN5                     (0x0013A20041B9FD22)
#define SEN_MSG_MAC_SN6                     (0x0013A20041BA26F1)
#define SEN_MSG_MAC_SNx                     (0x0013A20041B9F864)
#define SEN_MSG_MAC_RPI                     (0x0013A20041B9FD07)
#define SEN_MSG_MAC_DEF                     (0x0013A20041B9E77A)
/* Get the IDs (SL) */
#define SEN_MSG_ID_CH                       (SEN_MSG_MAC_SL(SEN_MSG_MAC_CH))
#define SEN_MSG_ID_SN1                      (SEN_MSG_MAC_SL(SEN_MSG_MAC_SN1))
#define SEN_MSG_ID_SN2                      (SEN_MSG_MAC_SL(SEN_MSG_MAC_SN2))
#define SEN_MSG_ID_SN3                      (SEN_MSG_MAC_SL(SEN_MSG_MAC_SN3))
#define SEN_MSG_ID_SN4                      (SEN_MSG_MAC_SL(SEN_MSG_MAC_SN4))
#define SEN_MSG_ID_SN5                      (SEN_MSG_MAC_SL(SEN_MSG_MAC_SN5))
#define SEN_MSG_ID_SN6                      (SEN_MSG_MAC_SL(SEN_MSG_MAC_SN6))
#define SEN_MSG_ID_SP1                      (SEN_MSG_MAC_SL(SEN_MSG_MAC_SP1))
#define SEN_MSG_ID_SP2                      (SEN_MSG_MAC_SL(SEN_MSG_MAC_SP2))
#define SEN_MSG_ID_DEF                      (SEN_MSG_MAC_SL(SEN_MSG_MAC_DEF))


/***** ENUMERATION ****************************************************/
/*! Enumeration for the DS18x20 function return values */
typedef enum {
    /* Special */
    SEN_MSG_TYPE_IGNORE         = 0x00,     /**< Ignore value; 0 (dec) */
    SEN_MSG_TYPE_INCIDENTS      = 0x01,     /**< Incident counter; uint16; 1 (dec) */
    SEN_MSG_TYPE_REBOOT         = 0x02,     /**< Reboot source; float; 2 (dec) */
    /* Temperature (0x1.) */
    SEN_MSG_TYPE_TEMP_RES       = 0x10,     /**< RESERVED; 16 (dec) */
    SEN_MSG_TYPE_TEMP_AIR       = 0x11,     /**< Air temperature; float; 17 (dec) */
    SEN_MSG_TYPE_TEMP_SOIL      = 0x12,     /**< Soil temperature; float; 18 (dec) */
    SEN_MSG_TYPE_TEMP_MCU       = 0x13,     /**< MCU core temperature; float; 19 (dec) */
    SEN_MSG_TYPE_TEMP_RADIO     = 0x14,     /**< Radio core temperature; float; 20 (dec) */
    SEN_MSG_TYPE_TEMP_SURFACE   = 0x15,     /**< MCU surface temperature; float; 21 (dec) */
    SEN_MSG_TYPE_TEMP_BOARD     = 0x16,     /**< Board surface temperature; float; 22 (dec) */
    /* Humidity (0x2.) */
    SEN_MSG_TYPE_HUMID_RES      = 0x20,     /**< RESERVED; 32 (dec) */
    SEN_MSG_TYPE_HUMID_AIR      = 0x21,     /**< Air relative humidity; float; 33 (dec) */
    SEN_MSG_TYPE_HUMID_SOIL     = 0x22,     /**< Soil humidity; float; 34 (dec) */
    /* Light intensity (0x3.) */
    SEN_MSG_TYPE_LUMI_RES       = 0x30,     /**< RESERVED; 48 (dec) */
    /* ... */
    /* Supply voltage (0xE.) */
    SEN_MSG_TYPE_VSS_RES        = 0xE0,     /**< RESERVED; 224 (dec) */
    SEN_MSG_TYPE_VSS_BAT        = 0xE1,     /**< Battery voltage; float; 225 (dec) */
    SEN_MSG_TYPE_VSS_MCU        = 0xE2,     /**< MCU supply voltage; float; 226 (dec) */
    SEN_MSG_TYPE_VSS_RADIO      = 0xE3,     /**< Radio supply voltage; float; 227 (dec) */
    /* Fault indicators (0xF.) */
    SEN_MSG_TYPE_X_RES          = 0xF0,     /**< RESERVED; 240 (dec) */
    SEN_MSG_TYPE_X_NT           = 0xF1,     /**< Normalized X_NT;    float; 241 (dec) */
    SEN_MSG_TYPE_X_VS           = 0xF2,     /**< Normalized X_VS;    float; 242 (dec) */
    SEN_MSG_TYPE_X_BAT          = 0xF3,     /**< Normalized X_BAT;   float; 243 (dec) */
    SEN_MSG_TYPE_X_ART          = 0xF4,     /**< Normalized X_ART;   float; 244 (dec) */
    SEN_MSG_TYPE_X_RST          = 0xF5,     /**< Normalized X_RST;   float; 245 (dec) */
    SEN_MSG_TYPE_X_IC           = 0xF6,     /**< Normalized X_IC;    float; 246 (dec) */
    SEN_MSG_TYPE_X_ADC          = 0xF7,     /**< Normalized X_ADC;   float; 247 (dec) */
    SEN_MSG_TYPE_X_USART        = 0xF8,     /**< Normalized X_USART; float; 248 (dec) */
} SEN_TYPE_t;


/***** STRUCTURES *****************************************************/
/*!
 * A structure to store a sensor type and value pair.
 */
typedef struct {
    uint8_t type;           /**< Sensor type (derived from SEN_TYPE_t) */
    uint16_t value;         /**< Sensor value (either uint16 or fixed16) */
} SEN_VALUE_t;

/*!
 * A structure to store the sensor message data.
 */
typedef struct {
    uint16_t time;          /**< timestamp (2 byte) */
    uint8_t cnt;            /**< Number of sensor values (1 byte) */
    SEN_VALUE_t values[SEN_MSG_NUM_MEASUREMENTS]; /**< Array of sensor values (3*NUM bytes) */
} SEN_MSG_t;


/***** UNIONS *********************************************************/
/*!
 * A union to access the sensor message via the structure or as byte array.
 */
/* Define for the message size [bytes] */
#define SEN_MSG_SIZE(VALUES)    (sizeof(uint16_t)+sizeof(uint8_t)+VALUES*sizeof(SEN_VALUE_t))
#define SEN_MSG_SIZE_MAX        SEN_MSG_SIZE(SEN_MSG_NUM_MEASUREMENTS)
typedef union {
    SEN_MSG_t struc;                /**< Access the message as a structure */
    uint8_t byte[SEN_MSG_SIZE_MAX]; /**< Access the message as a byte array */
} SEN_MSG_u;


/***** FUNCTION PROTOTYPES ********************************************/
void sen_msg_fill(SEN_MSG_u* msg, uint16_t time, uint8_t cnt, SEN_VALUE_t* values);
void sen_msg_read(SEN_MSG_u* msg, uint16_t* time, uint8_t* cnt, SEN_VALUE_t* values);

#endif // _ASNX_MSG_H_
