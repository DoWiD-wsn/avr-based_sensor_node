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
#if TIMER0_ENABLED
void (*_timer0_callback)() = NULL;
#endif
#if TIMER1_ENABLED
void (*_timer1_callback)() = NULL;
#endif
#if TIMER2_ENABLED
void (*_timer2_callback)() = NULL;
#endif
#if TIMER3_ENABLED
void (*_timer3_callback)() = NULL;
#endif


/***** FUNCTIONS **************************************************************/
#if TIMER0_ENABLED
/***
 * Calculate the ticks for TIMER0 for a time in microseconds (us).
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TIMER prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint8_t timer0_get_ticks_from_us(uint16_t us, TIMER_PRESCALER_t prescaler) {
    uint32_t timer;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER_PRESCALER_1:
            /* Using prescaler 1 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/1.0));
            break;
        case TIMER_PRESCALER_8:
            /* Using prescaler 8 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/8.0));
            break;
        case TIMER_PRESCALER_64:
            /* Using prescaler 64 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/64.0));
            break;
        case TIMER_PRESCALER_256:
            /* Using prescaler 256 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/256.0));
            break;
        case TIMER_PRESCALER_1024:
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
 * @param[in]   prescaler   TIMER prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint8_t timer0_get_ticks_from_ms(uint16_t ms, TIMER_PRESCALER_t prescaler) {
    uint32_t timer;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER_PRESCALER_1:
            /* Using prescaler 1 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/1.0));
            break;
        case TIMER_PRESCALER_8:
            /* Using prescaler 8 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/8.0));
            break;
        case TIMER_PRESCALER_64:
            /* Using prescaler 64 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/64.0));
            break;
        case TIMER_PRESCALER_256:
            /* Using prescaler 256 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/256.0));
            break;
        case TIMER_PRESCALER_1024:
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
    /* Stop the module */
    TCCR0B = (TCCR0B & 0xF8);
}


/***
 * Reset the TIMER0 module.
 ***/
void timer0_reset(void) {
    /* Reset all registers */
    TCCR0A = 0x00;
    TCCR0B = 0x00;
    TCNT0  = 0x00;
    OCR0A  = 0x00;
    OCR0B  = 0x00;
    TIMSK0 = 0x00;
    /* Reset the callback function pointer */
    _timer0_callback = NULL;
}


/***
 * Start TIMER0 with a given prescaler.
 *
 * @param[in]   prescaler   TIMER prescaler configuration
 ***/
void timer0_start(TIMER_PRESCALER_t prescaler) {
    /* Set the prescaler and start timer */
    TCCR0B = prescaler;
}


/***
 * Set the TCNT0 value.
 *
 * @param[in]   value       TCNT0 value to be set
 ***/
void timer0_set_tcnt(uint8_t value) {
    TCNT0 = value;
}


/***
 * Get the TCNT0 value.
 *
 * @return      TCNT0 value read
 ***/
uint8_t timer0_get_tcnt(void) {
    return TCNT0;
}


/***
 * Call a function after a defined number of ticks on TIMER0.
 *
 * @param[in]   ticks       Desired number of ticks
 * @param[in]   prescaler   TIMER prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer0_start_isr(uint8_t ticks, TIMER_PRESCALER_t prescaler, void (*func)()) {
    /* Check if a positive ticks value was given */
    if (ticks == 0) {
        return;
    }
    /* Disable timer during setup */
    TCCR0B = 0x00;
    /* Assign the user defined callback function */
    _timer0_callback = func;
    /* Set the counter value (top) */
    OCR0A = ticks;
    /* Run the timer in CTC mode */
    TCCR0A = _BV(WGM01);
    /* Reset the counter register */
    TCNT0 = 0;
    /* Enable the compare interrupt */
    TIMSK0 = _BV(OCIE0A);
    /* Set the prescaler and start timer */
    TCCR0B = prescaler;
}


/***
 * Call a function after a defined time in microseconds (us) on TIMER0.
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TIMER prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer0_start_isr_us(uint16_t us, TIMER_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = timer0_get_ticks_from_us(us, prescaler);
    /* Call the tick-based timer function */
    timer0_start_isr(ticks, prescaler, func);
}


/***
 * Call a function after a defined time in milliseconds (ms) on TIMER0.
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TIMER prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer0_start_isr_ms(uint16_t ms, TIMER_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = timer0_get_ticks_from_ms(ms, prescaler);
    /* Call the tick-based timer function */
    timer0_start_isr(ticks, prescaler, func);
}


/***
 * TIMER0 compare match interrupt.
 ***/
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


#if TIMER1_ENABLED
/***
 * Calculate the ticks for TIMER1 for a time in microseconds (us).
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TIMER prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint16_t timer1_get_ticks_from_us(uint16_t us, TIMER_PRESCALER_t prescaler) {
    uint32_t timer;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER_PRESCALER_1:
            /* Using prescaler 1 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/1.0));
            break;
        case TIMER_PRESCALER_8:
            /* Using prescaler 8 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/8.0));
            break;
        case TIMER_PRESCALER_64:
            /* Using prescaler 64 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/64.0));
            break;
        case TIMER_PRESCALER_256:
            /* Using prescaler 256 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/256.0));
            break;
        case TIMER_PRESCALER_1024:
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
 * @param[in]   prescaler   TIMER prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint16_t timer1_get_ticks_from_ms(uint16_t ms, TIMER_PRESCALER_t prescaler) {
    uint32_t timer;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER_PRESCALER_1:
            /* Using prescaler 1 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/1.0));
            break;
        case TIMER_PRESCALER_8:
            /* Using prescaler 8 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/8.0));
            break;
        case TIMER_PRESCALER_64:
            /* Using prescaler 64 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/64.0));
            break;
        case TIMER_PRESCALER_256:
            /* Using prescaler 256 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/256.0));
            break;
        case TIMER_PRESCALER_1024:
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
 * Stop the TIMER1 module.
 ***/
void timer1_stop(void) {
    /* Stop the module */
    TCCR1B = (TCCR1B & 0xF8);
}


/***
 * Reset the TIMER1 module.
 ***/
void timer1_reset(void) {
    /* Reset all registers */
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TCCR1C = 0x00;
    TCNT1  = 0x0000;
    OCR1A  = 0x0000;
    OCR1B  = 0x0000;
    ICR1   = 0x0000;
    TIMSK1 = 0x00;
    /* Reset the callback function pointer */
    _timer1_callback = NULL;
}


/***
 * Start TIMER1 with a given prescaler.
 *
 * @param[in]   prescaler   TIMER prescaler configuration
 ***/
void timer1_start(TIMER_PRESCALER_t prescaler) {
    /* Set the prescaler and start timer */
    TCCR1B = prescaler;
}


/***
 * Set the TCNT1 value.
 *
 * @param[in]   value       TCNT1 value to be set
 ***/
void timer1_set_tcnt(uint16_t value) {
    TCNT1 = value;
}


/***
 * Get the TCNT1 value.
 *
 * @return      TCNT1 value read
 ***/
uint16_t timer1_get_tcnt(void) {
    return TCNT1;
}


/***
 * Call a function after a defined number of ticks on TIMER1.
 *
 * @param[in]   ticks       Desired number of ticks
 * @param[in]   prescaler   TIMER prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer1_start_isr(uint16_t ticks, TIMER_PRESCALER_t prescaler, void (*func)()) {
    /* Check if a positive ticks value was given */
    if (ticks == 0) {
        return;
    }
    /* Disable timer during setup */
    TCCR1B = 0x00;
    /* Assign the user defined callback function */
    _timer1_callback = func;
    /* Set the counter value (top) */
    OCR1A = ticks;
    /* Run the timer in CTC mode */
    TCCR1A = 0;
    TCCR1B = _BV(WGM12);
    /* Reset the counter register */
    TCNT1 = 0;
    /* Enable the compare interrupt */
    TIMSK1 = _BV(OCIE1A);
    /* Set the prescaler and start timer */
    TCCR1B |= prescaler;
}


/***
 * Call a function after a defined time in microseconds (us) on TIMER1.
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TIMER prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer1_start_isr_us(uint16_t us, TIMER_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint16_t ticks = timer1_get_ticks_from_us(us, prescaler);
    /* Call the tick-based timer function */
    timer1_start_isr(ticks, prescaler, func);
}


/***
 * Call a function after a defined time in milliseconds (ms) on TIMER1.
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TIMER prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer1_start_isr_ms(uint16_t ms, TIMER_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint16_t ticks = timer1_get_ticks_from_ms(ms, prescaler);
    /* Call the tick-based timer function */
    timer1_start_isr(ticks, prescaler, func);
}


/***
 * TIMER1 compare match interrupt.
 ***/
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


#if TIMER2_ENABLED
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
 * Stop the TIMER2 module.
 ***/
void timer2_stop(void) {
    /* Stop the module */
    TCCR2B = (TCCR2B & 0xF8);
}


/***
 * Reset the TIMER2 module.
 ***/
void timer2_reset(void) {
    /* Reset all registers */
    TCCR2A = 0x00;
    TCCR2B = 0x00;
    TCNT2  = 0x00;
    OCR2A  = 0x00;
    OCR2B  = 0x00;
    ASSR   = 0x00;
    TIMSK2 = 0x00;
    /* Reset the callback function pointer */
    _timer2_callback = NULL;
}


/***
 * Start TIMER2 with a given prescaler.
 *
 * @param[in]   prescaler   TIMER2 prescaler configuration
 ***/
void timer2_start(TIMER2_PRESCALER_t prescaler) {
    /* Set the prescaler and start timer */
    TCCR2B = prescaler;
}


/***
 * Set the TCNT2 value.
 *
 * @param[in]   value       TCNT2 value to be set
 ***/
void timer2_set_tcnt(uint8_t value) {
    TCNT2 = value;
}


/***
 * Get the TCNT2 value.
 *
 * @return      TCNT2 value read
 ***/
uint8_t timer2_get_tcnt(void) {
    return TCNT2;
}


/***
 * Call a function after a defined number of ticks on TIMER2.
 *
 * @param[in]   ticks       Desired number of ticks
 * @param[in]   prescaler   TIMER2 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer2_start_isr(uint8_t ticks, TIMER2_PRESCALER_t prescaler, void (*func)()) {
    /* Check if a positive ticks value was given */
    if (ticks == 0) {
        return;
    }
    /* Disable timer during setup */
    TCCR2B = 0x00;
    /* Assign the user defined callback function */
    _timer2_callback = func;
    /* Set the counter value (top) */
    OCR2A = ticks;
    /* No asynchronous mode */
    ASSR = 0;
    /* Run the timer in CTC mode */
    TCCR2A = _BV(WGM21);
    /* Reset the counter register */
    TCNT2 = 0;
    /* Enable the compare interrupt */
    TIMSK2 = _BV(OCIE2A);
    /* Set the prescaler and start timer */
    TCCR2B = prescaler;
}


/***
 * Call a function after a defined time in microseconds (us) on TIMER2.
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TIMER2 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer2_start_isr_us(uint16_t us, TIMER2_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = timer2_get_ticks_from_us(us, prescaler);
    /* Call the tick-based timer function */
    timer2_start_isr(ticks, prescaler, func);
}


/***
 * Call a function after a defined time in milliseconds (ms) on TIMER2.
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TIMER2 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer2_start_isr_ms(uint16_t ms, TIMER2_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = timer2_get_ticks_from_ms(ms, prescaler);
    /* Call the tick-based timer function */
    timer2_start_isr(ticks, prescaler, func);
}


/***
 * TIMER2 compare match interrupt.
 ***/
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


#if TIMER3_ENABLED
/***
 * Calculate the ticks for TIMER3 for a time in microseconds (us).
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TIMER prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint16_t timer3_get_ticks_from_us(uint16_t us, TIMER_PRESCALER_t prescaler) {
    uint32_t timer;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER_PRESCALER_1:
            /* Using prescaler 1 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/1.0));
            break;
        case TIMER_PRESCALER_8:
            /* Using prescaler 8 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/8.0));
            break;
        case TIMER_PRESCALER_64:
            /* Using prescaler 64 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/64.0));
            break;
        case TIMER_PRESCALER_256:
            /* Using prescaler 256 */
            timer = (uint32_t)(((double)us*0.000001) * (F_CPU/256.0));
            break;
        case TIMER_PRESCALER_1024:
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
 * Calculate the ticks for TIMER3 for a time in milliseconds (ms).
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TIMER prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint16_t timer3_get_ticks_from_ms(uint16_t ms, TIMER_PRESCALER_t prescaler) {
    uint32_t timer;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER_PRESCALER_1:
            /* Using prescaler 1 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/1.0));
            break;
        case TIMER_PRESCALER_8:
            /* Using prescaler 8 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/8.0));
            break;
        case TIMER_PRESCALER_64:
            /* Using prescaler 64 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/64.0));
            break;
        case TIMER_PRESCALER_256:
            /* Using prescaler 256 */
            timer = (uint32_t)(((double)ms*0.001) * (F_CPU/256.0));
            break;
        case TIMER_PRESCALER_1024:
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
 * Stop the TIMER0 module.
 ***/
void timer3_stop(void) {
    /* Stop the module */
    TCCR3B = (TCCR3B & 0xF8);
}


/***
 * Reset the TIMER0 module.
 ***/
void timer3_reset(void) {
    /* Reset all registers */
    TCCR3A = 0x00;
    TCCR3B = 0x00;
    TCNT3  = 0x00;
    OCR3A  = 0x00;
    OCR3B  = 0x00;
    TIMSK3 = 0x00;
    /* Reset the callback function pointer */
    _timer3_callback = NULL;
}


/***
 * Start TIMER3 with a given prescaler.
 *
 * @param[in]   prescaler   TIMER prescaler configuration
 ***/
void timer3_start(TIMER_PRESCALER_t prescaler) {
    /* Set the prescaler and start timer */
    TCCR1B = prescaler;
}


/***
 * Set the TCNT3 value.
 *
 * @param[in]   value       TCNT3 value to be set
 ***/
void timer3_set_tcnt(uint16_t value) {
    TCNT3 = value;
}


/***
 * Get the TCNT3 value.
 *
 * @return      TCNT3 value read
 ***/
uint16_t timer3_get_tcnt(void) {
    return TCNT3;
}


/***
 * Call a function after a defined number of ticks on TIMER3.
 *
 * @param[in]   ticks       Desired number of ticks
 * @param[in]   prescaler   TIMER prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer3_start_isr(uint16_t ticks, TIMER_PRESCALER_t prescaler, void (*func)()) {
    /* Check if a positive ticks value was given */
    if (ticks == 0) {
        return;
    }
    /* Disable timer during setup */
    TCCR3B = 0x00;
    /* Assign the user defined callback function */
    _timer3_callback = func;
    /* Set the counter value (top) */
    OCR3A = ticks;
    /* Run the timer in CTC mode */
    TCCR3A = _BV(WGM31);
    /* Reset the counter register */
    TCNT3 = 0;
    /* Enable the compare interrupt */
    TIMSK3 = _BV(OCIE3A);
    /* Set the prescaler and start timer */
    TCCR3B = prescaler;
}


/***
 * Call a function after a defined time in microseconds (us) on TIMER3.
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TIMER prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer3_start_isr_us(uint16_t us, TIMER_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = timer3_get_ticks_from_us(us, prescaler);
    /* Call the tick-based timer function */
    timer3_start_isr(ticks, prescaler, func);
}


/***
 * Call a function after a defined time in milliseconds (ms) on TIMER3.
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TIMER prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void timer3_start_isr_ms(uint16_t ms, TIMER_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = timer3_get_ticks_from_ms(ms, prescaler);
    /* Call the tick-based timer function */
    timer3_start_isr(ticks, prescaler, func);
}


/***
 * TIMER0 compare match interrupt.
 ***/
#  if defined(__DOXYGEN__)
void TIMER3_COMPA_vect(void) {
#  else
ISR(TIMER3_COMPA_vect) {
#  endif
    /* If a callback is defined */
    if(_timer3_callback != NULL) {
        /* Call the callback function */
        _timer3_callback();
    }
}
#endif
