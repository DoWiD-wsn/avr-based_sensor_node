/*****
 * @brief   ASN(x) PCR85263 RTC library
 *
 * Library to support the PCR85263 RTC module.
 *
 * @file    /_asnx_lib_/rtc/pcf85263.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1.0 $
 * @date    $Date: 2021/05/10 $
 *****/

#ifndef _ASNX_PCR85263_H_
#define _ASNX_PCR85263_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>


/***** DEFINES ********************************************************/
/* I2C address */
#define PCR85263_I2C_ADDRESS            0x51


/***** ENUMERATION ****************************************************/
/* Enumeration for the PCR85263 function return values */
typedef enum {
    PCR85263_RET_ERROR      = -1,
    PCR85263_RET_OK         = 0
} PCR85263_RET_t;

/* Enumeration for the I2C register addresses */
typedef enum {
    /* RTC time and date registers */
    PCR85263_REG_DATETIME_100TH = 0x00,     /**< RTC time and date 100th seconds (0-99) */
    PCR85263_REG_DATETIME_SEC   = 0x01,     /**< RTC time and date seconds (0-59) */
    PCR85263_REG_DATETIME_MIN   = 0x02,     /**< RTC time and date minutes (0-59) */
    PCR85263_REG_DATETIME_HOUR  = 0x03,     /**< RTC time and date hours (0-23 / 1-12) */
    PCR85263_REG_DATETIME_DAY   = 0x04,     /**< RTC time and date days (1-31) */
    PCR85263_REG_DATETIME_DOTW  = 0x05,     /**< RTC time and date day-of-the-week (0-6) */
    PCR85263_REG_DATETIME_MONTH = 0x06,     /**< RTC time and date month (1-12) */
    PCR85263_REG_DATETIME_YEAR  = 0x07,     /**< RTC time and date years (0-99) */
    // TODO: continue!
} PCR85263_REG_t;


/***** STRUCTURES *****************************************************/


/***** FUNCTION PROTOTYPES ********************************************/


#endif // _ASNX_PCR85263_H_
