/**
 *  Header file for AVR TIMER functionality.
 */

#ifndef _AVR_TIMER_H_
#define _AVR_TIMER_H_

/***** INCLUDES ***************************************************************/
#include <stdint.h>


/***** MACROS *****************************************************************/
/*** BASIC CPU FREQUENCY ***/
#ifndef F_CPU
# warning "F_CPU not defined for \"timer.h\""
# define F_CPU 16000000UL
#endif
/*** ENABLE/DISABLE TIMER ***/
#define TIMER0_ENABLED                  (1)
#define TIMER1_ENABLED                  (1)
#define TIMER2_ENABLED                  (1)
/*** PRESCALER ***/
/* TCNT0 */
#define TIMER0_PRESCALER_NONE           (0)
#define TIMER0_PRESCALER_1              (1)
#define TIMER0_PRESCALER_8              (2)
#define TIMER0_PRESCALER_64             (3)
#define TIMER0_PRESCALER_256            (4)
#define TIMER0_PRESCALER_1024           (5)
/* TCNT1 */
#define TIMER1_PRESCALER_NONE           (0)
#define TIMER1_PRESCALER_1              (1)
#define TIMER1_PRESCALER_8              (2)
#define TIMER1_PRESCALER_64             (3)
#define TIMER1_PRESCALER_256            (4)
#define TIMER1_PRESCALER_1024           (5)
/* TCNT2 */
#define TIMER2_PRESCALER_NONE           (0)
#define TIMER2_PRESCALER_1              (1)
#define TIMER2_PRESCALER_8              (2)
#define TIMER2_PRESCALER_32             (3)
#define TIMER2_PRESCALER_64             (4)
#define TIMER2_PRESCALER_128            (5)
#define TIMER2_PRESCALER_256            (6)
#define TIMER2_PRESCALER_1024           (7)


/***** GLOBAL VARIABLES *******************************************************/


/***** ENUMERATION ************************************************************/


/***** STRUCTURES *************************************************************/


/***** FUNCTION PROTOTYPES ****************************************************/
/* stop timer */
void tcnt0_stop(void);
void tcnt1_stop(void);
void tcnt2_stop(void);
/* reset timer */
void tcnt0_reset(void);
void tcnt1_reset(void);
void tcnt2_reset(void);
/* ticks-based */
void tcnt0_start(uint8_t ticks, uint8_t prescaler, void (*func)());
void tcnt1_start(uint16_t ticks, uint8_t prescaler, void (*func)());
void tcnt2_start(uint8_t ticks, uint8_t prescaler, void (*func)());
/* time-based */
void tcnt0_start_us(uint16_t us, uint8_t prescaler, void (*func)());
void tcnt1_start_us(uint16_t us, uint8_t prescaler, void (*func)());
void tcnt2_start_us(uint16_t us, uint8_t prescaler, void (*func)());
void tcnt0_start_ms(uint16_t ms, uint8_t prescaler, void (*func)());
void tcnt1_start_ms(uint16_t ms, uint8_t prescaler, void (*func)());
void tcnt2_start_ms(uint16_t ms, uint8_t prescaler, void (*func)());


#endif // _AVR_TIMER_H_
