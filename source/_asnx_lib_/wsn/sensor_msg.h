/**
 *  Header file for SENOR MESSAGING functionality.
 */

#ifndef _SEN_MSG_H_
#define _SEN_MSG_H_

/***** INCLUDES ***************************************************************/
/* STD */
#include <stdint.h>
/* AVR */
#include <avr/io.h>


/***** DEFINITIONS ************************************************************/
/*** BYTE ORDER (per default, the values are send LSB first) ***/
#define SEN_MSG_REVERSE_BYTE_ORDER          (1)
#define SEN_MSG_SWAP_32BIT(value)           (__builtin_bswap32(value))

/*** SENSOR IDs (current network) ***/
/* Full 64-bit MACs */
#define SEN_MSG_MAC_CH                      (0x0013A20041B9DCDC)
#define SEN_MSG_MAC_SN1                     (0x0013A20041B9F805)
#define SEN_MSG_MAC_SN2                     (0x0013A20041B9FFC2)
#define SEN_MSG_MAC_SN3                     (0x0013A20041B9FFD8)
#define SEN_MSG_MAC_SN4                     (0x0013A20041B9FFDD)
#define SEN_MSG_MAC_SN5                     (0x0013A20041B9FD22)
#define SEN_MSG_MAC_SN6                     (0x0013A20041BA26F1)
#define SEN_MSG_MAC_SPARE                   (0x0013A20041B9F864)
#define SEN_MSG_MAC_DEF1                    (0x0013A20041B9E77A)
#define SEN_MSG_MAC_DEF2                    (0x0013A20041B9FD07)

/* Get SL & SH from MAC */
#define SEN_MSG_MAC_SH(mac)                 ((uint32_t)((mac>>32) & 0xFFFFFFFF))
#define SEN_MSG_MAC_SL(mac)                 ((uint32_t)(mac & 0xFFFFFFFF))
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



/*** TIMECODE ***/
#define SEN_MSG_TIME_STATIC                 (0x76543210)


/*** MEASUREMENT-TYPE ***/
enum MEASUREMENT_TYPE {
    /* RESERVED */
    SEN_MSG_TYPE_INCIDENTS      = 0x00,         // uint     0
    SEN_MSG_TYPE_REBOOT         = 0x01,         // uint     1
    /* TEMPERATURE (0x1.) */
    SEN_MSG_TYPE_TEMP_RES       = 0x10,         // float    16
    SEN_MSG_TYPE_TEMP_AIR       = 0x11,         // float    17
    SEN_MSG_TYPE_TEMP_SOIL      = 0x12,         // float    18
    SEN_MSG_TYPE_TEMP_MCU       = 0x13,         // float    19
    SEN_MSG_TYPE_TEMP_RADIO     = 0x14,         // float    20
    SEN_MSG_TYPE_TEMP_SURFACE   = 0x15,         // float    21
    /* HUMIDITY (0x2.) */
    SEN_MSG_TYPE_HUMID_RES      = 0x20,         // uint     32
    SEN_MSG_TYPE_HUMID_AIR      = 0x21,         // float    33
    SEN_MSG_TYPE_HUMID_SOIL     = 0x22,         // uint     34
    /* LIGHT INTENSITY (0x3.) */
    SEN_MSG_TYPE_LUMI_RES       = 0x30,         // uint     48
    /* ... */
    /* SUPPLY VOLTAGE (0xF.) */
    SEN_MSG_TYPE_VSS_RES        = 0xF0,         // uint     240
    SEN_MSG_TYPE_VSS_BAT        = 0xF1,         // uint     241
    SEN_MSG_TYPE_VSS_MCU        = 0xF2,         // uint     242
    SEN_MSG_TYPE_VSS_RADIO      = 0xF3,         // uint     243
};


/*** AVR SREG ***/
#define SEN_MSG_SREG                        (_SFR_IO8(0x3F))


/*** MESSAGE STRUCTURE ***/
/* Message structure */
typedef struct {
    uint32_t time;      // 4 byte
    uint8_t type;       // 1 byte
    float value;        // 4 byte
    uint8_t sreg;       // 1 byte
} msg_t;                // => 10 byte


/*** MESSAGE SIZE ***/
#define SEN_MSG_SIZE                        (sizeof(msg_t))


/*** ACCESS TO MESSAGE ***/
typedef union {
    msg_t struc;
    uint8_t byte[SEN_MSG_SIZE];
} sen_msg_t;


/***** FUNCTION PROTOTYPES ****************************************************/
void sen_msg_fill(sen_msg_t* msg, uint32_t time, uint8_t type, float value, uint8_t sreg);
void sen_msg_read(sen_msg_t* msg, uint32_t* time, uint8_t* type, float* value, uint8_t* sreg);


#endif // _SEN_MSG_H_
