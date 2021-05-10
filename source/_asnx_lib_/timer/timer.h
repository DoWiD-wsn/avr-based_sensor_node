/*****
 * @brief   ASN(x) timer library
 *
 * Library to support the use of the timer modules.
 *
 * @file    /_asnx_lib_/timer/timer.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1.0 $
 * @date    $Date: 2021/05/10 $
 *****/

#ifndef _ASNX_TIMER_H_
#define _ASNX_TIMER_H_

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
#define TIMER0_ENABLED           (1)
#define TIMER1_ENABLED           (1)
#define TIMER2_ENABLED           (1)


/***** ENUMERATION ****************************************************/
/* Enumeration for the available TIMER0 prescaler */
typedef enum {
    TIMER0_PRESCALER_NONE        = 0x00,     /**< Deactivated */
    TIMER0_PRESCALER_1           = 0x01,     /**< No prescaler */
    TIMER0_PRESCALER_8           = 0x02,     /**< Division factor 8 */
    TIMER0_PRESCALER_64          = 0x03,     /**< Division factor 64 */
    TIMER0_PRESCALER_256         = 0x04,     /**< Division factor 256 */
    TIMER0_PRESCALER_1024        = 0x05      /**< Division factor 1024 */
} TIMER0_PRESCALER_t;

/* Enumeration for the available TIMER1 prescaler */
typedef enum {
    TIMER1_PRESCALER_NONE        = 0x00,     /**< Deactivated */
    TIMER1_PRESCALER_1           = 0x01,     /**< No prescaler */
    TIMER1_PRESCALER_8           = 0x02,     /**< Division factor 8 */
    TIMER1_PRESCALER_64          = 0x03,     /**< Division factor 64 */
    TIMER1_PRESCALER_256         = 0x04,     /**< Division factor 256 */
    TIMER1_PRESCALER_1024        = 0x05      /**< Division factor 1024 */
} TIMER1_PRESCALER_t;

/* Enumeration for the available TIMER2 prescaler */
typedef enum {
    TIMER2_PRESCALER_NONE        = 0x00,     /**< Deactivated */
    TIMER2_PRESCALER_1           = 0x01,     /**< No prescaler */
    TIMER2_PRESCALER_8           = 0x02,     /**< Division factor 8 */
    TIMER2_PRESCALER_32          = 0x03,     /**< Division factor 32 */
    TIMER2_PRESCALER_64          = 0x04,     /**< Division factor 64 */
    TIMER2_PRESCALER_128         = 0x05,     /**< Division factor 128 */
    TIMER2_PRESCALER_256         = 0x06,     /**< Division factor 256 */
    TIMER2_PRESCALER_1024        = 0x07      /**< Division factor 1024 */
} TIMER2_PRESCALER_t;


/***** FUNCTION PROTOTYPES ********************************************/
/* stop timer */
void timer0_stop(void);
void timer1_stop(void);
void timer2_stop(void);
/* reset timer */
void timer0_reset(void);
void timer1_reset(void);
void timer2_reset(void);
/* ticks-based */
void timer0_start(uint8_t ticks, TIMER0_PRESCALER_t prescaler, void (*func)());
void timer1_start(uint16_t ticks, TIMER1_PRESCALER_t prescaler, void (*func)());
void timer2_start(uint8_t ticks, TIMER2_PRESCALER_t prescaler, void (*func)());
/* time-based */
void timer0_start_us(uint16_t us, TIMER0_PRESCALER_t prescaler, void (*func)());
void timer1_start_us(uint16_t us, TIMER1_PRESCALER_t prescaler, void (*func)());
void timer2_start_us(uint16_t us, TIMER2_PRESCALER_t prescaler, void (*func)());
void timer0_start_ms(uint16_t ms, TIMER0_PRESCALER_t prescaler, void (*func)());
void timer1_start_ms(uint16_t ms, TIMER1_PRESCALER_t prescaler, void (*func)());
void timer2_start_ms(uint16_t ms, TIMER2_PRESCALER_t prescaler, void (*func)());


#endif // _ASNX_TIMER_H_
