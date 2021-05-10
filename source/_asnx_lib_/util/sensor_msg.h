/*****
 * @brief   ASN(x) sensor network message library
 *
 * Library for functionality regarding the messaging of sensor values.
 *
 * @file    /_asnx_lib_/util/sensor_msg.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1.0 $
 * @date    $Date: 2021/05/10 $
 *****/

#ifndef _ASNX_MSG_H_
#define _ASNX_MSG_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>


/***** DEFINES ********************************************************/
/* Number of sensor measurements per message */
#ifndef SEN_MSG_NUM_MEASUREMENTS
#  define SEN_MSG_NUM_MEASUREMENTS            (13)
#endif

/* Get SL & SH from 64-bit MAC */
#define SEN_MSG_MAC_SH(mac)                 ((uint32_t)((mac>>32) & 0xFFFFFFFF))
#define SEN_MSG_MAC_SL(mac)                 ((uint32_t)(mac & 0xFFFFFFFF))

/***
 * Sensor node IDs (DW's testbed)
 * @todo    Update with your own Xbee MACs
 ***/
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
/* Enumeration for the DS18x20 function return values */
typedef enum {
    /* Special */
    SEN_MSG_TYPE_IGNORE         = 0x00,     /**< Ignore value; uint16; 0 (dec) */
    SEN_MSG_TYPE_INCIDENTS      = 0x01,     /**< Incident counter; uint16; 1 (dec) */
    SEN_MSG_TYPE_REBOOT         = 0x02,     /**< Reboot source; uint16; 2 (dec) */
    /* Temperature (0x1.) */
    SEN_MSG_TYPE_TEMP_RES       = 0x10,     /**< RESERVED; float; 16 (dec) */
    SEN_MSG_TYPE_TEMP_AIR       = 0x11,     /**< Air temperature; float; 17 (dec) */
    SEN_MSG_TYPE_TEMP_SOIL      = 0x12,     /**< Soil temperature; float; 18 (dec) */
    SEN_MSG_TYPE_TEMP_MCU       = 0x13,     /**< MCU core temperature; float; 19 (dec) */
    SEN_MSG_TYPE_TEMP_RADIO     = 0x14,     /**< Radio core temperature; float; 20 (dec) */
    SEN_MSG_TYPE_TEMP_SURFACE   = 0x15,     /**< MCU surface temperature; float; 21 (dec) */
    SEN_MSG_TYPE_TEMP_BOARD     = 0x16,     /**< Board surface temperature; float; 22 (dec) */
    /* Humidity (0x2.) */
    SEN_MSG_TYPE_HUMID_RES      = 0x20,     /**< RESERVED; float; 32 (dec) */
    SEN_MSG_TYPE_HUMID_AIR      = 0x21,     /**< Air relative humidity; float; 33 (dec) */
    SEN_MSG_TYPE_HUMID_SOIL     = 0x22,     /**< Soil humidity; float; 34 (dec) */
    /* Light intensity (0x3.) */
    SEN_MSG_TYPE_LUMI_RES       = 0x30,     /**< RESERVED; uint16; 48 (dec) */
    /* ... */
    /* Supply voltage (0xE.) */
    SEN_MSG_TYPE_VSS_RES        = 0xE0,     /**< RESERVED; float; 224 (dec) */
    SEN_MSG_TYPE_VSS_BAT        = 0xE1,     /**< Battery voltage; float; 225 (dec) */
    SEN_MSG_TYPE_VSS_MCU        = 0xE2,     /**< MCU supply voltage; float; 226 (dec) */
    SEN_MSG_TYPE_VSS_RADIO      = 0xE3,     /**< Radio supply voltage; float; 227 (dec) */
    /* Self-check (0xF.) */
    SEN_MSG_TYPE_CHK_RES        = 0xF0,     /**< RESERVED; uint16; 240 (dec) */
    SEN_MSG_TYPE_CHK_ADC        = 0xF1,     /**< ADC self check; uint16; 241 (dec) */
    SEN_MSG_TYPE_CHK_UART       = 0xF2,     /**< UART self check; uint16; 242 (dec) */
} SEN_TYPE_t;


/***** STRUCTURES *****************************************************/
/***
 * A structure to store a sensor type and value pair.
 ***/
typedef struct {
    uint8_t type;           /**< Sensor type (derived from SEN_TYPE_t) */
    uint16_t value;         /**< Sensor value (either uint16 or fixed16) */
} SEN_VALUE_t;

/***
 * A structure to store the sensor message data.
 ***/
typedef struct {
    uint16_t time;          /**< timestamp (2 byte) */
    uint8_t cnt;            /**< Number of sensor values (1 byte) */
    SEN_VALUE_t values[SEN_MSG_NUM_MEASUREMENTS]; /**< Array of sensor values (3*NUM bytes) */
} SEN_MSG_t;


/***** UNIONS *********************************************************/
/***
 * A union to access the sensor message via the structure or as byte array.
 ***/
/* Define for the message size [bytes] */
#define SEN_MSG_SIZE    (sizeof(uint16_t)+sizeof(uint8_t)+SEN_MSG_NUM_MEASUREMENTS*sizeof(SEN_VALUE_t))
typedef union {
    SEN_MSG_t struc;
    uint8_t byte[SEN_MSG_SIZE];
} SEN_MSG_u;


/***** FUNCTION PROTOTYPES ********************************************/
void sen_msg_fill(SEN_MSG_u* msg, uint16_t time, uint8_t cnt, SEN_VALUE_t* values);
void sen_msg_read(SEN_MSG_u* msg, uint16_t* time, uint8_t* cnt, SEN_VALUE_t* values);

#endif // _ASNX_MSG_H_
