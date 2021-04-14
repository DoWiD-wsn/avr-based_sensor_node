/*****
 * @brief   ASN(x) timer/counter library
 *
 * Library to support the use of the timer/counter module.
 *
 * @file    /_asnx_lib_/tcnt/tcnt.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/14 $
 *****/


/***** INCLUDES *******************************************************/
#include "tcnt.h"
/*** STD ***/
#include <stddef.h>
/*** AVR ***/
#include <avr/io.h>
#include <avr/interrupt.h>


/***** GLOBAL VARIABLES ***********************************************/
/* Callback function pointer for ISR */
void (*_tcnt0_callback)() = NULL;
void (*_tcnt1_callback)() = NULL;
void (*_tcnt2_callback)() = NULL;


/***** LOCAL FUNCTION PROTOTYPES **************************************/
uint8_t tcnt0_get_ticks_from_us(uint16_t us, TCNT0_PRESCALER_t prescaler);
uint8_t tcnt0_get_ticks_from_ms(uint16_t ms, TCNT0_PRESCALER_t prescaler);
uint16_t tcnt1_get_ticks_from_us(uint16_t us, TCNT1_PRESCALER_t prescaler);
uint16_t tcnt1_get_ticks_from_ms(uint16_t ms, TCNT1_PRESCALER_t prescaler);
uint8_t tcnt2_get_ticks_from_us(uint16_t us, TCNT2_PRESCALER_t prescaler);
uint8_t tcnt2_get_ticks_from_ms(uint16_t ms, TCNT2_PRESCALER_t prescaler);


/***** FUNCTIONS **************************************************************/
/***
 * Calculate the ticks for TCNT0 for a time in microseconds (us).
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TCNT0 prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint8_t tcnt0_get_ticks_from_us(uint16_t us, TCNT0_PRESCALER_t prescaler) {
    uint32_t tcnt;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TCNT0_PRESCALER_1:
            /* Using prescaler 1 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/1.0));
            break;
        case TCNT0_PRESCALER_8:
            /* Using prescaler 8 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/8.0));
            break;
        case TCNT0_PRESCALER_64:
            /* Using prescaler 64 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/64.0));
            break;
        case TCNT0_PRESCALER_256:
            /* Using prescaler 256 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/256.0));
            break;
        case TCNT0_PRESCALER_1024:
            /* Using prescaler 1024 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/1024.0));
            break;
        default:
            /* No timer operation */
            return 0;
    }
    /* Check if a valid tick value was derived */
    if((tcnt>0x00) && (tcnt<=0xFF)) {
        /* Valid result */
        return (uint8_t)tcnt;
    } else {
        /* Invalid result */
        return 0;
    }
}


/***
 * Calculate the ticks for TCNT0 for a time in milliseconds (ms).
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TCNT0 prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint8_t tcnt0_get_ticks_from_ms(uint16_t ms, TCNT0_PRESCALER_t prescaler) {
    uint32_t tcnt;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TCNT0_PRESCALER_1:
            /* Using prescaler 1 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/1.0));
            break;
        case TCNT0_PRESCALER_8:
            /* Using prescaler 8 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/8.0));
            break;
        case TCNT0_PRESCALER_64:
            /* Using prescaler 64 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/64.0));
            break;
        case TCNT0_PRESCALER_256:
            /* Using prescaler 256 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/256.0));
            break;
        case TCNT0_PRESCALER_1024:
            /* Using prescaler 1024 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/1024.0));
            break;
        default:
            /* No timer operation */
            return 0;
    }
    /* Check if a valid tick value was derived */
    if((tcnt>0x00) && (tcnt<=0xFF)) {
        /* Valid result */
        return (uint8_t)tcnt;
    } else {
        /* Invalid result */
        return 0;
    }
}


/***
 * Calculate the ticks for TCNT1 for a time in microseconds (us).
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TCNT1 prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint16_t tcnt1_get_ticks_from_us(uint16_t us, TCNT1_PRESCALER_t prescaler) {
    uint32_t tcnt;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TCNT1_PRESCALER_1:
            /* Using prescaler 1 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/1.0));
            break;
        case TCNT1_PRESCALER_8:
            /* Using prescaler 8 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/8.0));
            break;
        case TCNT1_PRESCALER_64:
            /* Using prescaler 64 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/64.0));
            break;
        case TCNT1_PRESCALER_256:
            /* Using prescaler 256 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/256.0));
            break;
        case TCNT1_PRESCALER_1024:
            /* Using prescaler 1024 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/1024.0));
            break;
        default:
            /* No timer operation */
            return 0;
    }
    /* Check if a valid tick value was derived */
    if((tcnt>0x0000) && (tcnt<=0xFFFF)) {
        /* Valid result */
        return (uint16_t)tcnt;
    } else {
        /* Invalid result */
        return 0;
    }
}


/***
 * Calculate the ticks for TCNT1 for a time in milliseconds (ms).
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TCNT1 prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint16_t tcnt1_get_ticks_from_ms(uint16_t ms, TCNT1_PRESCALER_t prescaler) {
    uint32_t tcnt;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TCNT1_PRESCALER_1:
            /* Using prescaler 1 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/1.0));
            break;
        case TCNT1_PRESCALER_8:
            /* Using prescaler 8 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/8.0));
            break;
        case TCNT1_PRESCALER_64:
            /* Using prescaler 64 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/64.0));
            break;
        case TCNT1_PRESCALER_256:
            /* Using prescaler 256 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/256.0));
            break;
        case TCNT1_PRESCALER_1024:
            /* Using prescaler 1024 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/1024.0));
            break;
        default:
            /* No timer operation */
            return 0;
    }
    /* Check if a valid tick value was derived */
    if((tcnt>0x0000) && (tcnt<=0xFFFF)) {
        /* Valid result */
        return (uint16_t)tcnt;
    } else {
        /* Invalid result */
        return 0;
    }
}


/***
 * Calculate the ticks for TCNT2 for a time in microseconds (us).
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TCNT2 prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint8_t tcnt2_get_ticks_from_us(uint16_t us, TCNT2_PRESCALER_t prescaler) {
    uint32_t tcnt;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TCNT2_PRESCALER_1:
            /* Using prescaler 1 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/1.0));
            break;
        case TCNT2_PRESCALER_8:
            /* Using prescaler 8 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/8.0));
            break;
        case TCNT2_PRESCALER_32:
            /* Using prescaler 32 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/32.0));
            break;
        case TCNT2_PRESCALER_64:
            /* Using prescaler 64 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/64.0));
            break;
        case TCNT2_PRESCALER_128:
            /* Using prescaler 128 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/128.0));
            break;
        case TCNT2_PRESCALER_256:
            /* Using prescaler 256 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/256.0));
            break;
        case TCNT2_PRESCALER_1024:
            /* Using prescaler 1024 */
            tcnt = (uint32_t)(((double)us*0.000001) * (F_CPU/1024.0));
            break;
        default:
            /* No timer operation */
            return 0;
    }
    /* Check if a valid tick value was derived */
    if((tcnt>0x00) && (tcnt<=0xFF)) {
        /* Valid result */
        return (uint8_t)tcnt;
    } else {
        /* Invalid result */
        return 0;
    }
}


/***
 * Calculate the ticks for TCNT2 for a time in milliseconds (ms).
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TCNT2 prescaler configuration
 * @return      Resulting number of ticks (nearest)
 ***/
uint8_t tcnt2_get_ticks_from_ms(uint16_t ms, TCNT2_PRESCALER_t prescaler) {
    uint32_t tcnt;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TCNT2_PRESCALER_1:
            /* Using prescaler 1 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/1.0));
            break;
        case TCNT2_PRESCALER_8:
            /* Using prescaler 8 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/8.0));
            break;
        case TCNT2_PRESCALER_32:
            /* Using prescaler 32 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/32.0));
            break;
        case TCNT2_PRESCALER_64:
            /* Using prescaler 64 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/64.0));
            break;
        case TCNT2_PRESCALER_128:
            /* Using prescaler 128 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/128.0));
            break;
        case TCNT2_PRESCALER_256:
            /* Using prescaler 256 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/256.0));
            break;
        case TCNT2_PRESCALER_1024:
            /* Using prescaler 1024 */
            tcnt = (uint32_t)(((double)ms*0.001) * (F_CPU/1024.0));
            break;
        default:
            /* No timer operation */
            return 0;
    }
    /* Check if a valid tick value was derived */
    if((tcnt>0x00) && (tcnt<=0xFF)) {
        /* Valid result */
        return (uint8_t)tcnt;
    } else {
        /* Invalid result */
        return 0;
    }
}


/***
 * Stop the TCNT0 module.
 ***/
void tcnt0_stop(void) {
    /* Disable the module */
    TIMSK0 &= ~(_BV(OCIE0A));
    /* Reset the callback function pointer */
    _tcnt0_callback = NULL;
    /* Reset all registers */
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0 = 0;
    OCR0A = 0;
}


/***
 * Stop the TCNT1 module.
 ***/
void tcnt1_stop(void) {
    /* Disable the module */
    TIMSK1 &= ~(_BV(OCIE1A));
    /* Reset the callback function pointer */
    _tcnt1_callback = NULL;
    /* Reset all registers */
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 0;
}


/***
 * Stop the TCNT2 module.
 ***/
void tcnt2_stop(void) {
    /* Disable the module */
    TIMSK2 &= ~(_BV(OCIE2A));
    /* Reset the callback function pointer */
    _tcnt2_callback = NULL;
    /* Reset all registers */
    TCCR2A = 0;
    TCCR2B = 0;
    ASSR = 0;
    TCNT2 = 0;
    OCR2A = 0;
}


/***
 * Reset the TCNT0 module.
 ***/
void tcnt0_reset(void) {
    /* Reset all registers */
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0 = 0;
    OCR0A = 0;
}


/***
 * Reset the TCNT1 module.
 ***/
void tcnt1_reset(void) {
    /* Reset all registers */
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 0;
}


/***
 * Reset the TCNT2 module.
 ***/
void tcnt2_reset(void) {
    /* Reset all registers */
    TCCR2A = 0;
    TCCR2B = 0;
    ASSR = 0;
    TCNT2 = 0;
    OCR2A = 0;
}


/***
 * Call a function after a defined number of ticks on TCNT0.
 *
 * @param[in]   ticks       Desired number of ticks
 * @param[in]   prescaler   TCNT0 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void tcnt0_start(uint8_t ticks, TCNT0_PRESCALER_t prescaler, void (*func)()) {
    /* Check if a positive ticks value was given */
    if (ticks == 0) {
        return;
    }
    /* During timer setup -> disable compare interrupt */
    TIMSK0 &= ~(_BV(OCIE0A));
    /* Assign the user defined callback function */
    _tcnt0_callback = func;
    /* Set the counter value (top) */
    OCR0A = ticks;
    /* Run the tcnt in CTC mode */
    TCCR0A = _BV(WGM01);
    /* Set the prescaler */
    TCCR0B = prescaler;
    /* Reset the counter register */
    TCNT0 = 0;
    /* Enable the compare interrupt */
    TIMSK0 |= _BV(OCIE0A);
}


/***
 * Call a function after a defined number of ticks on TCNT1.
 *
 * @param[in]   ticks       Desired number of ticks
 * @param[in]   prescaler   TCNT1 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void tcnt1_start(uint16_t ticks, TCNT1_PRESCALER_t prescaler, void (*func)()) {
    /* Check if a positive ticks value was given */
    if (ticks == 0) {
        return;
    }
    /* During timer setup -> disable compare interrupt */
    TIMSK1 &= ~(_BV(OCIE1A));
    /* Assign the user defined callback function */
    _tcnt1_callback = func;
    /* Set the counter value (top) */
    OCR1A = ticks;
    /* Run the tcnt in CTC mode and set the prescaler */
    TCCR1A = 0;
    TCCR1B = prescaler | _BV(WGM12);
    /* Reset the counter register */
    TCNT1 = 0;
    /* Enable the compare interrupt */
    TIMSK1 |= _BV(OCIE1A);
}


/***
 * Call a function after a defined number of ticks on TCNT2.
 *
 * @param[in]   ticks       Desired number of ticks
 * @param[in]   prescaler   TCNT2 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void tcnt2_start(uint8_t ticks, TCNT2_PRESCALER_t prescaler, void (*func)()) {
    /* Check if a positive ticks value was given */
    if (ticks == 0) {
        return;
    }
    /* During timer setup -> disable compare interrupt */
    TIMSK2 &= ~(_BV(OCIE2A));
    /* Assign the user defined callback function */
    _tcnt2_callback = func;
    /* Set the counter value (top) */
    OCR2A = ticks;
    /* No asynchronous mode */
    ASSR = 0;
    /* Run the tcnt in CTC mode */
    TCCR2A = _BV(WGM21);
    /* Set the prescaler */
    TCCR2B = prescaler;
    /* Reset the counter register */
    TCNT2 = 0;
    /* Enable the compare interrupt */
    TIMSK2 |= _BV(OCIE2A);
}


/***
 * Call a function after a defined time in microseconds (us) on TCNT0.
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TCNT0 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void tcnt0_start_us(uint16_t us, TCNT0_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = tcnt0_get_ticks_from_us(us, prescaler);
    /* Call the tick-based timer function */
    tcnt0_start(ticks, prescaler, func);
}


/***
 * Call a function after a defined time in microseconds (us) on TCNT1.
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TCNT1 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void tcnt1_start_us(uint16_t us, TCNT1_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint16_t ticks = tcnt1_get_ticks_from_us(us, prescaler);
    /* Call the tick-based timer function */
    tcnt1_start(ticks, prescaler, func);
}


/***
 * Call a function after a defined time in microseconds (us) on TCNT2.
 *
 * @param[in]   us          Desired time in microseconds (us)
 * @param[in]   prescaler   TCNT2 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void tcnt2_start_us(uint16_t us, TCNT2_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = tcnt2_get_ticks_from_us(us, prescaler);
    /* Call the tick-based timer function */
    tcnt2_start(ticks, prescaler, func);
}


/***
 * Call a function after a defined time in milliseconds (ms) on TCNT0.
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TCNT0 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void tcnt0_start_ms(uint16_t ms, TCNT0_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = tcnt0_get_ticks_from_ms(ms, prescaler);
    /* Call the tick-based timer function */
    tcnt0_start(ticks, prescaler, func);
}


/***
 * Call a function after a defined time in milliseconds (ms) on TCNT1.
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TCNT1 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void tcnt1_start_ms(uint16_t ms, TCNT1_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint16_t ticks = tcnt1_get_ticks_from_ms(ms, prescaler);
    /* Call the tick-based timer function */
    tcnt1_start(ticks, prescaler, func);
}


/***
 * Call a function after a defined time in milliseconds (ms) on TCNT2.
 *
 * @param[in]   ms          Desired time in milliseconds (ms)
 * @param[in]   prescaler   TCNT2 prescaler configuration
 * @param[in]   func        Callback function pointer
 ***/
void tcnt2_start_ms(uint16_t ms, TCNT2_PRESCALER_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = tcnt2_get_ticks_from_ms(ms, prescaler);
    /* Call the tick-based timer function */
    tcnt2_start(ticks, prescaler, func);
}


/***
 * TCNT0 data register empty interrupt.
 ***/
#ifdef TIMER0_ENABLED
#  if defined(__DOXYGEN__)
void TIMER0_COMPA_vect(void) {
#  else
ISR(TIMER0_COMPA_vect) {
#  endif
    /* If a callback is defined */
    if(_tcnt0_callback != NULL) {
        /* Call the callback function */
        _tcnt0_callback();
    }
}
#endif


/***
 * TCNT1 data register empty interrupt.
 ***/
#ifdef TIMER1_ENABLED
#  if defined(__DOXYGEN__)
void TIMER1_COMPA_vect(void) {
#  else
ISR(TIMER1_COMPA_vect) {
#  endif
    /* If a callback is defined */
    if(_tcnt1_callback != NULL) {
        /* Call the callback function */
        _tcnt1_callback();
    }
}
#endif


/***
 * TCNT2 data register empty interrupt.
 ***/
#ifdef TIMER2_ENABLED
#  if defined(__DOXYGEN__)
void TIMER2_COMPA_vect(void) {
#  else
ISR(TIMER2_COMPA_vect) {
#  endif
    /* If a callback is defined */
    if(_tcnt2_callback != NULL) {
        /* Call the callback function */
        _tcnt2_callback();
    }
}
#endif
