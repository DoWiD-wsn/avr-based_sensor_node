/*****
 * @brief   ASN(x) timer/counter library
 *
 * Library to support the use of the timer/counter module.
 *
 * @file    /_asnx_lib_/tcnt/tcnt.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/14 $
 *****/

#ifndef _ASNX_TCNT_H_
#define _ASNX_TCNT_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>


/***** DEFINES ********************************************************/
/*** CPU frequency (F_CPU) ***/
#ifndef F_CPU
# warning "F_CPU not defined for \"timer.h\""
# define F_CPU 4000000UL
#endif
/*** Enable (1) or disable (0) timer modules ***/
#define TCNT0_ENABLED           (1)
#define TCNT1_ENABLED           (1)
#define TCNT2_ENABLED           (1)
/*** Prescaler ***/
/* Enumeration for the available TCNT0 prescaler */
typedef enum {
    TCNT0_PRESCALER_NONE        = 0x00,     /**< Deactivated */
    TCNT0_PRESCALER_1           = 0x01,     /**< No prescaler */
    TCNT0_PRESCALER_8           = 0x02,     /**< Division factor 8 */
    TCNT0_PRESCALER_64          = 0x03,     /**< Division factor 64 */
    TCNT0_PRESCALER_256         = 0x04,     /**< Division factor 256 */
    TCNT0_PRESCALER_1024        = 0x05      /**< Division factor 1024 */
} TCNT0_PRESCALER_t;
/* Enumeration for the available TCNT1 prescaler */
typedef enum {
    TCNT1_PRESCALER_NONE        = 0x00,     /**< Deactivated */
    TCNT1_PRESCALER_1           = 0x01,     /**< No prescaler */
    TCNT1_PRESCALER_8           = 0x02,     /**< Division factor 8 */
    TCNT1_PRESCALER_64          = 0x03,     /**< Division factor 64 */
    TCNT1_PRESCALER_256         = 0x04,     /**< Division factor 256 */
    TCNT1_PRESCALER_1024        = 0x05      /**< Division factor 1024 */
} TCNT1_PRESCALER_t;
/* Enumeration for the available TCNT2 prescaler */
typedef enum {
    TCNT2_PRESCALER_NONE        = 0x00,     /**< Deactivated */
    TCNT2_PRESCALER_1           = 0x01,     /**< No prescaler */
    TCNT2_PRESCALER_8           = 0x02,     /**< Division factor 8 */
    TCNT2_PRESCALER_32          = 0x03,     /**< Division factor 32 */
    TCNT2_PRESCALER_64          = 0x04,     /**< Division factor 64 */
    TCNT2_PRESCALER_128         = 0x05,     /**< Division factor 128 */
    TCNT2_PRESCALER_256         = 0x06,     /**< Division factor 256 */
    TCNT2_PRESCALER_1024        = 0x07      /**< Division factor 1024 */
} TCNT2_PRESCALER_t;


/***** FUNCTION PROTOTYPES ********************************************/
/* stop timer */
void tcnt0_stop(void);
void tcnt1_stop(void);
void tcnt2_stop(void);
/* reset timer */
void tcnt0_reset(void);
void tcnt1_reset(void);
void tcnt2_reset(void);
/* ticks-based */
void tcnt0_start(uint8_t ticks, TCNT0_PRESCALER_t prescaler, void (*func)());
void tcnt1_start(uint16_t ticks, TCNT1_PRESCALER_t prescaler, void (*func)());
void tcnt2_start(uint8_t ticks, TCNT2_PRESCALER_t prescaler, void (*func)());
/* time-based */
void tcnt0_start_us(uint16_t us, TCNT0_PRESCALER_t prescaler, void (*func)());
void tcnt1_start_us(uint16_t us, TCNT1_PRESCALER_t prescaler, void (*func)());
void tcnt2_start_us(uint16_t us, TCNT2_PRESCALER_t prescaler, void (*func)());
void tcnt0_start_ms(uint16_t ms, TCNT0_PRESCALER_t prescaler, void (*func)());
void tcnt1_start_ms(uint16_t ms, TCNT1_PRESCALER_t prescaler, void (*func)());
void tcnt2_start_ms(uint16_t ms, TCNT2_PRESCALER_t prescaler, void (*func)());


#endif // _ASNX_TCNT_H_
