/**
 *  Source file for AVR TIMER functionality.
 */

/***** INCLUDES ***************************************************************/
/* STD */
#include <stddef.h>
/* AVR */
#include <avr/io.h>
#include <avr/interrupt.h>
/* OWN */
#include "timer/timer.h"
#include "util/fuses.h"


/***** GLOBAL VARIABLES *******************************************************/
/*** CALLBACK FUNCTIONS ***/
void (*_tcnt0_callback)() = NULL;
void (*_tcnt1_callback)() = NULL;
void (*_tcnt2_callback)() = NULL;


/***** LOCAL FUNCTION PROTOTYPES **********************************************/
uint8_t timer0_get_ticks_from_us(uint16_t us, uint8_t prescaler);
uint8_t timer0_get_ticks_from_ms(uint16_t ms, uint8_t prescaler);
uint16_t timer1_get_ticks_from_us(uint16_t us, uint8_t prescaler);
uint16_t timer1_get_ticks_from_ms(uint16_t ms, uint8_t prescaler);
uint8_t timer2_get_ticks_from_us(uint16_t us, uint8_t prescaler);
uint8_t timer2_get_ticks_from_ms(uint16_t ms, uint8_t prescaler);


/***** INTERRUPT SERVICE ROUTINE (ISR) ****************************************/
/*
 * TCNT0 ISR
 */
#ifdef TIMER0_ENABLED
ISR(TIMER0_COMPA_vect) {
    /* If a callback is defined */
    if(_tcnt0_callback != NULL) {
        /* Call the callback function */
        _tcnt0_callback();
    }
}
#endif


/*
 * TCNT1 ISR
 */
#ifdef TIMER1_ENABLED
ISR(TIMER1_COMPA_vect) {
    /* If a callback is defined */
    if(_tcnt1_callback != NULL) {
        /* Call the callback function */
        _tcnt1_callback();
    }
}
#endif


/*
 * TCNT2 ISR
 */
#ifdef TIMER2_ENABLED
ISR(TIMER2_COMPA_vect) {
    /* If a callback is defined */
    if(_tcnt2_callback != NULL) {
        /* Call the callback function */
        _tcnt2_callback();
    }
}
#endif


/***** FUNCTIONS **************************************************************/
/*** HELPER FUNCTIONS *****************/
/*
 * Calculate the ticks for timer0 for a time in us
 */
uint8_t timer0_get_ticks_from_us(uint16_t us, uint8_t prescaler) {
    uint32_t tcnt;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER0_PRESCALER_1:
            /* Using prescaler 1 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/1.0));
            break;
        case TIMER0_PRESCALER_8:
            /* Using prescaler 8 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/8.0));
            break;
        case TIMER0_PRESCALER_64:
            /* Using prescaler 64 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/64.0));
            break;
        case TIMER0_PRESCALER_256:
            /* Using prescaler 256 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/256.0));
            break;
        case TIMER0_PRESCALER_1024:
            /* Using prescaler 1024 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/1024.0));
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


/*
 * Calculate the ticks for timer0 for a time in ms
 */
uint8_t timer0_get_ticks_from_ms(uint16_t ms, uint8_t prescaler) {
    uint32_t tcnt;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER0_PRESCALER_1:
            /* Using prescaler 1 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/1.0));
            break;
        case TIMER0_PRESCALER_8:
            /* Using prescaler 8 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/8.0));
            break;
        case TIMER0_PRESCALER_64:
            /* Using prescaler 64 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/64.0));
            break;
        case TIMER0_PRESCALER_256:
            /* Using prescaler 256 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/256.0));
            break;
        case TIMER0_PRESCALER_1024:
            /* Using prescaler 1024 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/1024.0));
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


/*
 * Calculate the ticks for timer2 for a time in us
 */
uint8_t timer2_get_ticks_from_us(uint16_t us, uint8_t prescaler) {
    uint32_t tcnt;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER2_PRESCALER_1:
            /* Using prescaler 1 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/1.0));
            break;
        case TIMER2_PRESCALER_8:
            /* Using prescaler 8 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/8.0));
            break;
        case TIMER2_PRESCALER_32:
            /* Using prescaler 32 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/32.0));
            break;
        case TIMER2_PRESCALER_64:
            /* Using prescaler 64 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/64.0));
            break;
        case TIMER2_PRESCALER_128:
            /* Using prescaler 128 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/128.0));
            break;
        case TIMER2_PRESCALER_256:
            /* Using prescaler 256 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/256.0));
            break;
        case TIMER2_PRESCALER_1024:
            /* Using prescaler 1024 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/1024.0));
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


/*
 * Calculate the ticks for timer2 for a time in ms
 */
uint8_t timer2_get_ticks_from_ms(uint16_t ms, uint8_t prescaler) {
    uint32_t tcnt;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER2_PRESCALER_1:
            /* Using prescaler 1 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/1.0));
            break;
        case TIMER2_PRESCALER_8:
            /* Using prescaler 8 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/8.0));
            break;
        case TIMER2_PRESCALER_32:
            /* Using prescaler 32 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/32.0));
            break;
        case TIMER2_PRESCALER_64:
            /* Using prescaler 64 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/64.0));
            break;
        case TIMER2_PRESCALER_128:
            /* Using prescaler 128 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/128.0));
            break;
        case TIMER2_PRESCALER_256:
            /* Using prescaler 256 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/256.0));
            break;
        case TIMER2_PRESCALER_1024:
            /* Using prescaler 1024 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/1024.0));
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


/*
 * Calculate the ticks for timer for a time in us
 */
uint16_t timer1_get_ticks_from_us(uint16_t us, uint8_t prescaler) {
    uint32_t tcnt;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER1_PRESCALER_1:
            /* Using prescaler 1 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/1.0));
            break;
        case TIMER1_PRESCALER_8:
            /* Using prescaler 8 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/8.0));
            break;
        case TIMER1_PRESCALER_64:
            /* Using prescaler 64 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/64.0));
            break;
        case TIMER1_PRESCALER_256:
            /* Using prescaler 256 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/256.0));
            break;
        case TIMER1_PRESCALER_1024:
            /* Using prescaler 1024 */
            tcnt = (uint32_t)(((double)us*0.000001) * ((double)fuses_get_fcpu()/1024.0));
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


/*
 * Calculate the ticks for timer1 for a time in ms
 */
uint16_t timer1_get_ticks_from_ms(uint16_t ms, uint8_t prescaler) {
    uint32_t tcnt;
    /* Get the peripheral clock */
    switch(prescaler) {
        case TIMER1_PRESCALER_1:
            /* Using prescaler 1 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/1.0));
            break;
        case TIMER1_PRESCALER_8:
            /* Using prescaler 8 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/8.0));
            break;
        case TIMER1_PRESCALER_64:
            /* Using prescaler 64 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/64.0));
            break;
        case TIMER1_PRESCALER_256:
            /* Using prescaler 256 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/256.0));
            break;
        case TIMER1_PRESCALER_1024:
            /* Using prescaler 1024 */
            tcnt = (uint32_t)(((double)ms*0.001) * ((double)fuses_get_fcpu()/1024.0));
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


/*
 * Stop the timer0
 */
void tcnt0_stop(void) {
    /* Disable the tcnt */
    TIMSK0 &= ~(_BV(OCIE0A));
    _tcnt0_callback = NULL;
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0 = 0;
    OCR0A = 0;
}


/*
 * Stop the timer1
 */
void tcnt1_stop(void) {
    /* Disable the tcnt */
    TIMSK1 &= ~(_BV(OCIE1A));
    _tcnt1_callback = NULL;
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 0;
}


/*
 * Stop the timer2
 */
void tcnt2_stop(void) {
    /* Disable the tcnt */
    TIMSK2 &= ~(_BV(OCIE2A));
    _tcnt2_callback = NULL;
    TCCR2A = 0;
    TCCR2B = 0;
    ASSR = 0;
    TCNT2 = 0;
    OCR2A = 0;
}


/*
 * Reset the timer0
 */
void tcnt0_reset(void) {
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0 = 0;
    OCR0A = 0;
}


/*
 * Reset the timer1
 */
void tcnt1_reset(void) {
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 0;
}


/*
 * Reset the timer2
 */
void tcnt2_reset(void) {
    TCCR2A = 0;
    TCCR2B = 0;
    ASSR = 0;
    TCNT2 = 0;
    OCR2A = 0;
}


/*** TIME MODE FUNCTIONS **************/
/*
 * Start the timer0 counter with callback function
 */
void tcnt0_start(uint8_t ticks, uint8_t prescaler, void (*func)()) {
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


/*
 * Start the timer1 counter with callback function
 */
void tcnt1_start(uint16_t ticks, uint8_t prescaler, void (*func)()) {
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


/*
 * Start the timer2 counter with callback function
 */
void tcnt2_start(uint8_t ticks, uint8_t prescaler, void (*func)()) {
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


/*
 * Start the timer0 counter with callback function (time in us)
 */
void tcnt0_start_us(uint16_t us, uint8_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = timer0_get_ticks_from_us(us, prescaler);
    /* Call the tick-based timer function */
    tcnt0_start(ticks, prescaler, func);
}


/*
 * Start the timer1 counter with callback function (time in us)
 */
void tcnt1_start_us(uint16_t us, uint8_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint16_t ticks = timer1_get_ticks_from_us(us, prescaler);
    /* Call the tick-based timer function */
    tcnt1_start(ticks, prescaler, func);
}


/*
 * Start the timer2 counter with callback function (time in us)
 */
void tcnt2_start_us(uint16_t us, uint8_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = timer2_get_ticks_from_us(us, prescaler);
    /* Call the tick-based timer function */
    tcnt2_start(ticks, prescaler, func);
}


/*
 * Start the timer0 counter with callback function (time in ms)
 */
void tcnt0_start_ms(uint16_t ms, uint8_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = timer0_get_ticks_from_ms(ms, prescaler);
    /* Call the tick-based timer function */
    tcnt0_start(ticks, prescaler, func);
}


/*
 * Start the timer1 counter with callback function (time in ms)
 */
void tcnt1_start_ms(uint16_t ms, uint8_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint16_t ticks = timer1_get_ticks_from_ms(ms, prescaler);
    /* Call the tick-based timer function */
    tcnt1_start(ticks, prescaler, func);
}


/*
 * Start the timer2 counter with callback function (time in ms)
 */
void tcnt2_start_ms(uint16_t ms, uint8_t prescaler, void (*func)()) {
    /* Get the ticks from time */
    uint8_t ticks = timer2_get_ticks_from_ms(ms, prescaler);
    /* Call the tick-based timer function */
    tcnt2_start(ticks, prescaler, func);
}
