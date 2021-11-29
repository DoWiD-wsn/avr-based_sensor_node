/*!
 * @brief   ASN(x) timer library -- header file
 *
 * Library to support the use of the timer modules.
 *
 * @file    /_asnx_lib_/timer/timer.h
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 */

#ifndef _ASNX_TIMER_H_
#define _ASNX_TIMER_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
#include <stddef.h>
/*** AVR ***/
#include <avr/io.h>
#include <avr/interrupt.h>


/***** DEFINES ********************************************************/
/*! CPU frequency (F_CPU) */
#ifndef F_CPU
# warning "F_CPU not defined for \"timer.h\""
# define F_CPU 4000000UL
#endif
/*** Enable (1) or disable (0) timer modules ***/
#define TIMER0_ENABLED           (1)        /**< Enable timer0 */
#define TIMER1_ENABLED           (1)        /**< Enable timer1 */
#define TIMER2_ENABLED           (1)        /**< Enable timer2 */
#define TIMER3_ENABLED           (1)        /**< Enable timer3 */


/***** ENUMERATION ****************************************************/
#if (TIMER0_ENABLED || TIMER1_ENABLED || TIMER3_ENABLED)
/*! Enumeration for the available TIMER0 prescaler */
typedef enum {
    TIMER_PRESCALER_NONE        = 0x00,     /**< Deactivated */
    TIMER_PRESCALER_1           = 0x01,     /**< No prescaler */
    TIMER_PRESCALER_8           = 0x02,     /**< Division factor 8 */
    TIMER_PRESCALER_64          = 0x03,     /**< Division factor 64 */
    TIMER_PRESCALER_256         = 0x04,     /**< Division factor 256 */
    TIMER_PRESCALER_1024        = 0x05,     /**< Division factor 1024 */
    TIMER_PRESCALER_EXT_FALLING = 0x06,     /**< External source on T0 (falling edge) */
    TIMER_PRESCALER_EXT_RISING  = 0x07      /**< External source on T0 (rising edge) */
} TIMER_PRESCALER_t;
#endif

#if TIMER2_ENABLED
/*! Enumeration for the available TIMER2 prescaler */
typedef enum {
    TIMER2_PRESCALER_NONE       = 0x00,     /**< Deactivated */
    TIMER2_PRESCALER_1          = 0x01,     /**< No prescaler */
    TIMER2_PRESCALER_8          = 0x02,     /**< Division factor 8 */
    TIMER2_PRESCALER_32         = 0x03,     /**< Division factor 32 */
    TIMER2_PRESCALER_64         = 0x04,     /**< Division factor 64 */
    TIMER2_PRESCALER_128        = 0x05,     /**< Division factor 128 */
    TIMER2_PRESCALER_256        = 0x06,     /**< Division factor 256 */
    TIMER2_PRESCALER_1024       = 0x07      /**< Division factor 1024 */
} TIMER2_PRESCALER_t;
#endif


/***** FUNCTION PROTOTYPES ********************************************/
#if TIMER0_ENABLED
uint8_t timer0_get_ticks_from_us(uint16_t us, TIMER_PRESCALER_t prescaler);
uint8_t timer0_get_ticks_from_ms(uint16_t ms, TIMER_PRESCALER_t prescaler);
void timer0_stop(void);
void timer0_reset(void);
void timer0_start(TIMER_PRESCALER_t prescaler);
void timer0_set_tcnt(uint8_t value);
uint8_t timer0_get_tcnt(void);
void timer0_start_isr(uint8_t ticks, TIMER_PRESCALER_t prescaler, void (*func)(void));
void timer0_start_isr_us(uint16_t us, TIMER_PRESCALER_t prescaler, void (*func)(void));
void timer0_start_isr_ms(uint16_t ms, TIMER_PRESCALER_t prescaler, void (*func)(void));
#endif
#if TIMER1_ENABLED
uint16_t timer1_get_ticks_from_us(uint16_t us, TIMER_PRESCALER_t prescaler);
uint16_t timer1_get_ticks_from_ms(uint16_t ms, TIMER_PRESCALER_t prescaler);
void timer1_stop(void);
void timer1_reset(void);
void timer1_start(TIMER_PRESCALER_t prescaler);
void timer1_set_tcnt(uint16_t value);
uint16_t timer1_get_tcnt(void);
void timer1_start_isr(uint16_t ticks, TIMER_PRESCALER_t prescaler, void (*func)(void));
void timer1_start_isr_us(uint16_t us, TIMER_PRESCALER_t prescaler, void (*func)(void));
void timer1_start_isr_ms(uint16_t ms, TIMER_PRESCALER_t prescaler, void (*func)(void));
#endif
#if TIMER2_ENABLED
uint8_t timer2_get_ticks_from_us(uint16_t us, TIMER2_PRESCALER_t prescaler);
uint8_t timer2_get_ticks_from_ms(uint16_t ms, TIMER2_PRESCALER_t prescaler);
void timer2_stop(void);
void timer2_reset(void);
void timer2_start(TIMER2_PRESCALER_t prescaler);
void timer2_set_tcnt(uint8_t value);
uint8_t timer2_get_tcnt(void);
void timer2_start_isr(uint8_t ticks, TIMER2_PRESCALER_t prescaler, void (*func)(void));
void timer2_start_isr_us(uint16_t us, TIMER2_PRESCALER_t prescaler, void (*func)(void));
void timer2_start_isr_ms(uint16_t ms, TIMER2_PRESCALER_t prescaler, void (*func)(void));
#endif
#if TIMER3_ENABLED
uint16_t timer3_get_ticks_from_us(uint16_t us, TIMER_PRESCALER_t prescaler);
uint16_t timer3_get_ticks_from_ms(uint16_t ms, TIMER_PRESCALER_t prescaler);
void timer3_stop(void);
void timer3_reset(void);
void timer3_start(TIMER_PRESCALER_t prescaler);
void timer3_set_tcnt(uint16_t value);
uint16_t timer3_get_tcnt(void);
void timer3_start_isr(uint16_t ticks, TIMER_PRESCALER_t prescaler, void (*func)(void));
void timer3_start_isr_us(uint16_t us, TIMER_PRESCALER_t prescaler, void (*func)(void));
void timer3_start_isr_ms(uint16_t ms, TIMER_PRESCALER_t prescaler, void (*func)(void));
#endif

#endif // _ASNX_TIMER_H_
