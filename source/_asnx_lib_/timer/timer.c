/*****
 * @brief   ASN(x) timer/counter library
 *
 * Library to support the use of the timer/counter module.
 *
 * @file    /_asnx_lib_/timer/timer.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1.0 $
 * @date    $Date: 2021/05/10 $
 *****/


/***** INCLUDES *******************************************************/
#include "timer.h"
/*** STD ***/
#include <stddef.h>
/*** AVR ***/
#include <avr/io.h>
#include <avr/interrupt.h>


/***** GLOBAL VARIABLES ***********************************************/
/* Callback function pointer for ISR */
void (*_timer0_callback)() = NULL;
void (*_timer1_callback)() = NULL;
void (*_timer2_callback)() = NULL;


/***** LOCAL FUNCTION PROTOTYPES **************************************/
uint8_t timer0_get_ticks_from_us(uint16_t us, TIMER0_PRESCALER_t prescaler);
uint8_t timer0_get_ticks_from_ms(uint16_t ms, TIMER0_PRESCALER_t prescaler);
uint16_t timer1_get_ticks_from_us(uint16_t us, TIMER1_PRESCALER_t prescaler);
uint16_t timer1_get_ticks_from_ms(uint16_t ms, TIMER1_PRESCALER_t prescaler);
uint8_t timer2_get_ticks_from_us(uint16_t us, TIMER2_PRESCALER_t prescaler);
uint8_t timer2_get_ticks_from_ms(uint16_t ms, TIMER2_PRESCALER_t prescaler);


/***** FUNCTIONS **************************************************************/
/***
 * Calculate the ticks for TIMER0 for a time in microseconds (us).
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TIMER0 prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint8_t timer0_get_ticks_from_us(uint16_t us, TIMER0_PRESCALER_t prescaler) {
    uint32_t timer;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER0_PRESCALER_1:
            /* Using prescaler 1 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/1.0));
            break;
        case TIMER0_PRESCALER_8:
            /* Using prescaler 8 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/8.0));
            break;
        case TIMER0_PRESCALER_64:
            /* Using prescaler 64 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/64.0));
            break;
        case TIMER0_PRESCALER_256:
            /* Using prescaler 256 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/256.0));
            break;
        case TIMER0_PRESCALER_1024:
            /* Using prescaler 1024 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/1024.0));
            break;
        default:
            /* No timer operation */
            return 0;
    }
    /* Check if a valid tick value was derived */
    if((timer>0x00) && (timer<=0xFF)) {
        /* Valid result */
        return (uint8_t)timer;
    } else {
        /* Invalid result */
        return 0;
    }
}


/***
 * Calculate the ticks for TIMER0 for a time in milliseconds (ms).
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TIMER0 prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint8_t timer0_get_ticks_from_ms(uint16_t ms, TIMER0_PRESCALER_t prescaler) {
    uint32_t timer;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER0_PRESCALER_1:
            /* Using prescaler 1 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/1.0));
            break;
        case TIMER0_PRESCALER_8:
            /* Using prescaler 8 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/8.0));
            break;
        case TIMER0_PRESCALER_64:
            /* Using prescaler 64 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/64.0));
            break;
        case TIMER0_PRESCALER_256:
            /* Using prescaler 256 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/256.0));
            break;
        case TIMER0_PRESCALER_1024:
            /* Using prescaler 1024 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/1024.0));
            break;
        default:
            /* No timer operation */
            return 0;
    }
    /* Check if a valid tick value was derived */
    if((timer>0x00) && (timer<=0xFF)) {
        /* Valid result */
        return (uint8_t)timer;
    } else {
        /* Invalid result */
        return 0;
    }
}


/***
 * Calculate the ticks for TIMER1 for a time in microseconds (us).
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TIMER1 prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint16_t timer1_get_ticks_from_us(uint16_t us, TIMER1_PRESCALER_t prescaler) {
    uint32_t timer;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER1_PRESCALER_1:
            /* Using prescaler 1 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/1.0));
            break;
        case TIMER1_PRESCALER_8:
            /* Using prescaler 8 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/8.0));
            break;
        case TIMER1_PRESCALER_64:
            /* Using prescaler 64 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/64.0));
            break;
        case TIMER1_PRESCALER_256:
            /* Using prescaler 256 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/256.0));
            break;
        case TIMER1_PRESCALER_1024:
            /* Using prescaler 1024 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/1024.0));
            break;
        default:
            /* No timer operation */
            return 0;
    }
    /* Check if a valid tick value was derived */
    if((timer>0x0000) && (timer<=0xFFFF)) {
        /* Valid result */
        return (uint16_t)timer;
    } else {
        /* Invalid result */
        return 0;
    }
}


/***
 * Calculate the ticks for TIMER1 for a time in milliseconds (ms).
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TIMER1 prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint16_t timer1_get_ticks_from_ms(uint16_t ms, TIMER1_PRESCALER_t prescaler) {
    uint32_t timer;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER1_PRESCALER_1:
            /* Using prescaler 1 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/1.0));
            break;
        case TIMER1_PRESCALER_8:
            /* Using prescaler 8 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/8.0));
            break;
        case TIMER1_PRESCALER_64:
            /* Using prescaler 64 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/64.0));
            break;
        case TIMER1_PRESCALER_256:
            /* Using prescaler 256 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/256.0));
            break;
        case TIMER1_PRESCALER_1024:
            /* Using prescaler 1024 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/1024.0));
            break;
        default:
            /* No timer operation */
            return 0;
    }
    /* Check if a valid tick value was derived */
    if((timer>0x0000) && (timer<=0xFFFF)) {
        /* Valid result */
        return (uint16_t)timer;
    } else {
        /* Invalid result */
        return 0;
    }
}


/***
 * Calculate the ticks for TIMER2 for a time in microseconds (us).
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TIMER2 prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint8_t timer2_get_ticks_from_us(uint16_t us, TIMER2_PRESCALER_t prescaler) {
    uint32_t timer;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER2_PRESCALER_1:
            /* Using prescaler 1 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/1.0));
            break;
        case TIMER2_PRESCALER_8:
            /* Using prescaler 8 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/8.0));
            break;
        case TIMER2_PRESCALER_32:
            /* Using prescaler 32 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/32.0));
            break;
        case TIMER2_PRESCALER_64:
            /* Using prescaler 64 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/64.0));
            break;
        case TIMER2_PRESCALER_128:
            /* Using prescaler 128 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/128.0));
            break;
        case TIMER2_PRESCALER_256:
            /* Using prescaler 256 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/256.0));
            break;
        case TIMER2_PRESCALER_1024:
            /* Using prescaler 1024 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/1024.0));
            break;
        default:
            /* No timer operation */
            return 0;
    }
    /* Check if a valid tick value was derived */
    if((timer>0x00) && (timer<=0xFF)) {
        /* Valid result */
        return (uint8_t)timer;
    } else {
        /* Invalid result */
        return 0;
    }
}


/***
 * Calculate the ticks for TIMER2 for a time in milliseconds (ms).
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TIMER2 prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint8_t timer2_get_ticks_from_ms(uint16_t ms, TIMER2_PRESCALER_t prescaler) {
    uint32_t timer;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER2_PRESCALER_1:
            /* Using prescaler 1 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/1.0));
            break;
        case TIMER2_PRESCALER_8:
            /* Using prescaler 8 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/8.0));
            break;
        case TIMER2_PRESCALER_32:
            /* Using prescaler 32 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/32.0));
            break;
        case TIMER2_PRESCALER_64:
            /* Using prescaler 64 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/64.0));
            break;
        case TIMER2_PRESCALER_128:
            /* Using prescaler 128 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/128.0));
            break;
        case TIMER2_PRESCALER_256:
            /* Using prescaler 256 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/256.0));
            break;
        case TIMER2_PRESCALER_1024:
            /* Using prescaler 1024 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/1024.0));
            break;
        default:
            /* No timer operation */
            return 0;
    }
    /* Check if a valid tick value was derived */
    if((timer>0x00) && (timer<=0xFF)) {
        /* Valid result */
        return (uint8_t)timer;
    } else {
        /* Invalid result */
        return 0;
    }
}


/***
 * Stop the TIMER0 module.
 ***/
void timer0_stop(void) {
    /* Disable the module */
    TIMSK0 &= ~(_BV(OCIE0A));
    /* Reset the callback function pointer */
    _timer0_callback = NULL;
    /* Reset all registers */
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0 = 0;
    OCR0A = 0;
}


/***
 * Stop the TIMER1 module.
 ***/
void timer1_stop(void) {
    /* Disable the module */
    TIMSK1 &= ~(_BV(OCIE1A));
    /* Reset the callback function pointer */
    _timer1_callback = NULL;
    /* Reset all registers */
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 0;
}


/***
 * Stop the TIMER2 module.
 ***/
void timer2_stop(void) {
    /* Disable the module */
    TIMSK2 &= ~(_BV(OCIE2A));
    /* Reset the callback function pointer */
    _timer2_callback = NULL;
    /* Reset all registers */
    TCCR2A = 0;
    TCCR2B = 0;
    ASSR = 0;
    TCNT2 = 0;
    OCR2A = 0;
}


/***
 * Reset the TIMER0 module.
 ***/
void timer0_reset(void) {
    /* Reset all registers */
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0 = 0;
    OCR0A = 0;
}


/***
 * Reset the TIMER1 module.
 ***/
void timer1_reset(void) {
    /* Reset all registers */
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 0;
}


/***
 * Reset the TIMER2 module.
 ***/
void timer2_reset(void) {
    /* Reset all registers */
    TCCR2A = 0;
    TCCR2B = 0;
    ASSR = 0;
    TCNT2 = 0;
    OCR2A = 0;
}


/***
 * Call a function after a defined number of ticks on TIMER0.
 *
 * @param[in]   ticks       Desired number of ticks
 * @param[in]   prescaler   TIMER0 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer0_start(uint8_t ticks, TIMER0_PRESCALER_t prescaler, void (*func)()) {
    /* Check if a positive ticks value was given */
    if (ticks == 0) {
        return;
    }
    /* During timer setup -> disable compare interrupt */
    TIMSK0 &= ~(_BV(OCIE0A));
    /* Assign the user defined callback function */
    _timer0_callback = func;
    /* Set the counter value (top) */
    OCR0A = ticks;
    /* Run the timer in CTC mode */
    TCCR0A = _BV(WGM01);
    /* Set the prescaler */
    TCCR0B = prescaler;
    /* Reset the counter register */
    TCNT0 = 0;
    /* Enable the compare interrupt */
    TIMSK0 |= _BV(OCIE0A);
}


/***
 * Call a function after a defined number of ticks on TIMER1.
 *
 * @param[in]   ticks       Desired number of ticks
 * @param[in]   prescaler   TIMER1 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer1_start(uint16_t ticks, TIMER1_PRESCALER_t prescaler, void (*func)()) {
    /* Check if a positive ticks value was given */
    if (ticks == 0) {
        return;
    }
    /* During timer setup -> disable compare interrupt */
    TIMSK1 &= ~(_BV(OCIE1A));
    /* Assign the user defined callback function */
    _timer1_callback = func;
    /* Set the counter value (top) */
    OCR1A = ticks;
    /* Run the timer in CTC mode and set the prescaler */
    TCCR1A = 0;
    TCCR1B = prescaler | _BV(WGM12);
    /* Reset the counter register */
    TCNT1 = 0;
    /* Enable the compare interrupt */
    TIMSK1 |= _BV(OCIE1A);
}


/***
 * Call a function after a defined number of ticks on TIMER2.
 *
 * @param[in]   ticks       Desired number of ticks
 * @param[in]   prescaler   TIMER2 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer2_start(uint8_t ticks, TIMER2_PRESCALER_t prescaler, void (*func)()) {
    /* Check if a positive ticks value was given */
    if (ticks == 0) {
        return;
    }
    /* During timer setup -> disable compare interrupt */
    TIMSK2 &= ~(_BV(OCIE2A));
    /* Assign the user defined callback function */
    _timer2_callback = func;
    /* Set the counter value (top) */
    OCR2A = ticks;
    /* No asynchronous mode */
    ASSR = 0;
    /* Run the timer in CTC mode */
    TCCR2A = _BV(WGM21);
    /* Set the prescaler */
    TCCR2B = prescaler;
    /* Reset the counter register */
    TCNT2 = 0;
    /* Enable the compare interrupt */
    TIMSK2 |= _BV(OCIE2A);
}


/***
 * Call a function after a defined time in microseconds (us) on TIMER0.
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TIMER0 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer0_start_us(uint16_t us, TIMER0_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = timer0_get_ticks_from_us(us, prescaler);
    /* Call the tick-based timer function */
    timer0_start(ticks, prescaler, func);
}


/***
 * Call a function after a defined time in microseconds (us) on TIMER1.
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TIMER1 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer1_start_us(uint16_t us, TIMER1_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint16_t ticks = timer1_get_ticks_from_us(us, prescaler);
    /* Call the tick-based timer function */
    timer1_start(ticks, prescaler, func);
}


/***
 * Call a function after a defined time in microseconds (us) on TIMER2.
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TIMER2 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer2_start_us(uint16_t us, TIMER2_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = timer2_get_ticks_from_us(us, prescaler);
    /* Call the tick-based timer function */
    timer2_start(ticks, prescaler, func);
}


/***
 * Call a function after a defined time in milliseconds (ms) on TIMER0.
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TIMER0 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer0_start_ms(uint16_t ms, TIMER0_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = timer0_get_ticks_from_ms(ms, prescaler);
    /* Call the tick-based timer function */
    timer0_start(ticks, prescaler, func);
}


/***
 * Call a function after a defined time in milliseconds (ms) on TIMER1.
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TIMER1 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer1_start_ms(uint16_t ms, TIMER1_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint16_t ticks = timer1_get_ticks_from_ms(ms, prescaler);
    /* Call the tick-based timer function */
    timer1_start(ticks, prescaler, func);
}


/***
 * Call a function after a defined time in milliseconds (ms) on TIMER2.
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TIMER2 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer2_start_ms(uint16_t ms, TIMER2_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = timer2_get_ticks_from_ms(ms, prescaler);
    /* Call the tick-based timer function */
    timer2_start(ticks, prescaler, func);
}


/***
 * TIMER0 data register empty interrupt.
 ***/
#ifdef TIMER0_ENABLED
#  if defined(__DOXYGEN__)
void TIMER0_COMPA_vect(void) {
#  else
ISR(TIMER0_COMPA_vect) {
#  endif
    /* If a callback is defined */
    if(_timer0_callback != NULL) {
        /* Call the callback function */
        _timer0_callback();
    }
}
#endif


/***
 * TIMER1 data register empty interrupt.
 ***/
#ifdef TIMER1_ENABLED
#  if defined(__DOXYGEN__)
void TIMER1_COMPA_vect(void) {
#  else
ISR(TIMER1_COMPA_vect) {
#  endif
    /* If a callback is defined */
    if(_timer1_callback != NULL) {
        /* Call the callback function */
        _timer1_callback();
    }
}
#endif


/***
 * TIMER2 data register empty interrupt.
 ***/
#ifdef TIMER2_ENABLED
#  if defined(__DOXYGEN__)
void TIMER2_COMPA_vect(void) {
#  else
ISR(TIMER2_COMPA_vect) {
#  endif
    /* If a callback is defined */
    if(_timer2_callback != NULL) {
        /* Call the callback function */
        _timer2_callback();
    }
}
#endif
