/**
 *  Source file for AVR SYSTICK functionality.
 */

/***** INCLUDES ***************************************************************/
/* STD */
#include <stddef.h>
/* OWN */
#include "systick.h"
#include "timer.h"


/***** GLOBAL VARIABLES *******************************************************/
systick_t systime = {0,0,0};
/*** CALLBACK FUNCTIONS ***/
void (*_systick_cb_msec)() = NULL;
void (*_systick_cb_sec)() = NULL;
void (*_systick_cb_min)() = NULL;


/***** LOCAL FUNCTION PROTOTYPES **********************************************/
void systick_tick(void);


/***** FUNCTIONS **************************************************************/
/*
 * Initialization of the systick timer
 */
void systick_init(void) {
    /* Init timer0 to call the callback every 1ms */
    tcnt0_start_ms(1, TIMER0_PRESCALER_64, systick_tick);
}


/*
 * Deinitialization of the systick timer
 */
void systick_deinit(void) {
    /* Deinitialize (stop) timer0 */
    tcnt0_stop();
}


/*
 * Reset of the systick timer and data structure
 */
void systick_reset(void) {
    /* Deinit systick timer */
    systick_deinit();
    /* Reset data structure */
    systime.msec = 0;
    systime.sec = 0;
    systime.min = 0;
    /* Re-init systick timer */
    systick_init();
}


/*
 * Get the total number of ticks elapsed (ms)
 */
uint32_t systick_get_ticks(void) {
    /* Derive the total number of milli-seconds */
    return ((uint32_t)systime.msec + ((uint32_t)systime.sec*1000) + ((uint32_t)systime.min*1000*60));
}


/*
 * Get the number of msec elapsed
 */
uint16_t systick_get_msec(void) {
    /* Return the number of milli-seconds */
    return systime.msec;
}


/*
 * Get the number of sec elapsed
 */
uint8_t systick_get_sec(void) {
    /* Return the number of seconds */
    return systime.sec;
}


/*
 * Get the number of min elapsed
 */
uint16_t systick_get_min(void) {
    /* Return the number of minutes */
    return systime.min;
}


/*
 * ISR Callback function
 */
void systick_tick(void) {
    /* Increment number of milli-seconds */
    systime.msec++;
    /* Check if a second has passed */
    if(systime.msec >= 1000) {
        /* Increment number of seconds */
        systime.sec++;
        /* Reset milli-seconds */
        systime.msec = 0;
        /* Check if a minute has passed */
        if(systime.sec >= 60) {
            /* Increment number of seconds */
            systime.min++;
            /* Reset milli-seconds */
            systime.sec = 0;
            /* Check if min callback sould be called */
            if(_systick_cb_min != NULL) {
                /* Call the callback */
                _systick_cb_min();
            }
        }
        /* Check if sec callback sould be called */
        if(_systick_cb_sec != NULL) {
            /* Call the callback */
            _systick_cb_sec();
        }
    }
    /* Check if msec callback sould be called */
    if(_systick_cb_msec != NULL) {
        /* Call the callback */
        _systick_cb_msec();
    }
}


/*
 * Set a msec callback function
 */
void systick_set_callback_msec(void (*func)()) {
    /* Assign the user defined callback function */
    _systick_cb_msec = func;
}


/*
 * Set a sec callback function
 */
void systick_set_callback_sec(void (*func)()) {
    /* Assign the user defined callback function */
    _systick_cb_sec = func;
}


/*
 * Set a min callback function
 */
void systick_set_callback_min(void (*func)()) {
    /* Assign the user defined callback function */
    _systick_cb_min = func;
}


/*
 * Clear the msec callback function
 */
void systick_clear_callback_msec(void) {
    /* Assign the user defined callback function */
    _systick_cb_msec = NULL;
}


/*
 * Clear the sec callback function
 */
void systick_clear_callback_sec(void) {
    /* Assign the user defined callback function */
    _systick_cb_sec = NULL;
}


/*
 * Clear the min callback function
 */
void systick_clear_callback_min(void) {
    /* Assign the user defined callback function */
    _systick_cb_min = NULL;
}
