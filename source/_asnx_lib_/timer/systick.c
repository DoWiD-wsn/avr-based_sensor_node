/*!
 * @brief   ASN(x) systick library -- source file
 *
 * Library to enable a systick timer based on the TIMER0 module.
 *
 * @file    /_asnx_lib_/timer/systick.c
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 *
 * @note    The systick timer is stopped when the MCU enters specific sleep modes!
 */


/***** INCLUDES *******************************************************/
#include "systick.h"


/***** GLOBAL VARIABLES ***********************************************/
/*! Global struct for the elapsed time of the systick timer */
systick_t systime = {0,0,0,0,0};
/* Callback function pointer */
void (*_systick_cb_msec)(void) = NULL;  /**< msec function callback */
void (*_systick_cb_sec)(void) = NULL;   /**< sec function callback */
void (*_systick_cb_min)(void) = NULL;   /**< min function callback */
void (*_systick_cb_hour)(void) = NULL;  /**< hour function callback */


/***** LOCAL FUNCTION PROTOTYPES **************************************/
void systick_tick(void);


/***** FUNCTIONS ******************************************************/
/*!
 * Initialize the systick timer.
 */
void systick_init(void) {
    /* Init timer0 to call the callback every 1ms */
    timer0_start_isr_ms(1, TIMER_PRESCALER_64, systick_tick);
}


/*!
 * Deinitialization of the systick timer.
 */
void systick_deinit(void) {
    /* Deinitialize timer0 */
    timer0_reset();
}


/*!
 * Reset of the systick timer and data structure.
 */
void systick_reset(void) {
    /* Deinit systick timer */
    systick_deinit();
    /* Reset data structure */
    systime.msec = 0;
    systime.sec = 0;
    systime.min = 0;
    systime.hour = 0;
    systime.day = 0;
    /* Re-init systick timer */
    systick_init();
}


/*!
 * Get the total number of ticks elapsed (equals ms).
 * 
 * @return  Number of systick ticks elapsed
 */
uint64_t systick_get_ticks(void) {
    /* Derive the total number of milli-seconds */
    return ((uint64_t)systime.msec + ((uint64_t)systime.sec*1000) + ((uint64_t)systime.min*1000*60) + ((uint64_t)systime.hour*1000*60*60) + ((uint64_t)systime.day*1000*60*60*24));
}


/*!
 * Get the total number of milliseconds (ms) elapsed.
 * 
 * @return  Number of milliseconds (ms) elapsed
 */
uint16_t systick_get_msec(void) {
    /* Return the number of milli-seconds */
    return systime.msec;
}


/*!
 * Get the total number of seconds (sec) elapsed.
 * 
 * @return  Number of seconds (sec) elapsed
 */
uint8_t systick_get_sec(void) {
    /* Return the number of seconds */
    return systime.sec;
}


/*!
 * Get the total number of minutes (min) elapsed.
 * 
 * @return  Number of minutes (min) elapsed
 */
uint8_t systick_get_min(void) {
    /* Return the number of minutes */
    return systime.min;
}


/*!
 * Get the total number of hours (hour) elapsed.
 * 
 * @return  Number of hours (hour) elapsed
 */
uint8_t systick_get_hour(void) {
    /* Return the number of hours */
    return systime.hour;
}


/*!
 * Get the total number of days (day) elapsed.
 * 
 * @return  Number of days (day) elapsed
 */
uint16_t systick_get_day(void) {
    /* Return the number of days */
    return systime.day;
}


/*!
 * Systick timer callback function to be called by timer0 ISR.
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
            /* Increment number of minutes */
            systime.min++;
            /* Reset seconds */
            systime.sec = 0;
            /* Check if an hour has passed */
            if(systime.min >= 60) {
                /* Increment number of hours */
                systime.hour++;
                /* Reset minutes */
                systime.min = 0;
                /* Check if a day has passed */
                if(systime.hour >= 24) {
                    /* Increment number of days */
                    systime.day++;
                    /* Reset hours */
                    systime.hour = 0;
                }
                /* Check if hour callback should be called */
                if(_systick_cb_hour != NULL) {
                    /* Call the callback */
                    _systick_cb_hour();
                }
            }
            /* Check if min callback should be called */
            if(_systick_cb_min != NULL) {
                /* Call the callback */
                _systick_cb_min();
            }
        }
        /* Check if sec callback should be called */
        if(_systick_cb_sec != NULL) {
            /* Call the callback */
            _systick_cb_sec();
        }
    }
    /* Check if msec callback should be called */
    if(_systick_cb_msec != NULL) {
        /* Call the callback */
        _systick_cb_msec();
    }
}


/*!
 * Set a callback function to be called every elapsed millisecond (ms).
 *
 * @param[in]   func    Callback function pointer
 */
void systick_set_callback_msec(void (*func)(void)) {
    /* Assign the user defined callback function */
    _systick_cb_msec = func;
}


/*!
 * Set a callback function to be called every elapsed second (sec).
 *
 * @param[in]   func    Callback function pointer
 */
void systick_set_callback_sec(void (*func)(void)) {
    /* Assign the user defined callback function */
    _systick_cb_sec = func;
}


/*!
 * Set a callback function to be called every elapsed minute (min).
 *
 * @param[in]   func    Callback function pointer
 */
void systick_set_callback_min(void (*func)(void)) {
    /* Assign the user defined callback function */
    _systick_cb_min = func;
}


/*!
 * Set a callback function to be called every elapsed hour.
 *
 * @param[in]   func    Callback function pointer
 */
void systick_set_callback_hour(void (*func)(void)) {
    /* Assign the user defined callback function */
    _systick_cb_hour = func;
}


/*!
 * Clear the ms callback function.
 */
void systick_clear_callback_msec(void) {
    /* Assign the user defined callback function */
    _systick_cb_msec = NULL;
}


/*!
 * Clear the sec callback function.
 */
void systick_clear_callback_sec(void) {
    /* Assign the user defined callback function */
    _systick_cb_sec = NULL;
}


/*!
 * Clear the min callback function.
 */
void systick_clear_callback_min(void) {
    /* Assign the user defined callback function */
    _systick_cb_min = NULL;
}


/*!
 * Clear the hour callback function.
 */
void systick_clear_callback_hour(void) {
    /* Assign the user defined callback function */
    _systick_cb_hour = NULL;
}
