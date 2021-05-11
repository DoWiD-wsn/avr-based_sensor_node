/*****
 * @brief   ASN(x) PCR85263 RTC library
 *
 * Library to support the PCR85263 RTC module.
 *
 * @file    /_asnx_lib_/rtc/pcf85263.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1.0 $
 * @date    $Date: 2021/05/10 $
 *****/


/***** INCLUDES *******************************************************/
#include "pcf85263.h"


/***** LOCAL FUNCTION PROTOTYPES **************************************/
static inline uint8_t _bcd2dec(uint8_t value);
static inline uint8_t _dec2bcd(uint8_t value);


/***** FUNCTIONS ******************************************************/
/***
 * Convert a BCD-encoded value into a decimal value (0-99).
 * 
 * @param[in]   value   BCD-encoded value.
 * @return      Converted decimal value.
 ***/
static inline uint8_t _bcd2dec(uint8_t value) {
    /* Multiply high nibble by ten and add low nibble */
    return ((value&0xF0) >> 4) * 10 + (value&0x0F);
}


/***
 * Convert a decimal value into a BCD-encoded value (0-99).
 * 
 * @param[in]   value   Decimal value.
 * @return      Converted BCD-encoded value.
 ***/
static inline uint8_t _dec2bcd(uint8_t value) {
    /* Put number of tens to high nibble and unit digit to low nibble */
    return ((value/10) << 4) + (value%10);
}
