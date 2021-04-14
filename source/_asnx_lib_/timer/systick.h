/*****
 * @brief   ASN(x) systick library
 *
 * Library to enable a systick timer based on the TCNT0 module.
 *
 * @file    /_asnx_lib_/timer/systick.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/14 $
 *****/

#ifndef _ASNX_SYSTICK_H_
#define _ASNX_SYSTICK_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>


/***** STRUCTURES *****************************************************/
/***
 * A structure to store the elapsed time of the systick timer.
 ***/
typedef struct {
    uint16_t msec;      /**< Elapsed milliseconds (ms) */
    uint8_t sec;        /**< Elapsed seconds (sec) */
    uint16_t min;       /**< Elapsed minutes (min) */
} systick_t;


/***** GLOBAL VARIABLES ***********************************************/
/* Global struct for the elapsed time of the systick timer */
extern systick_t systime;


/***** FUNCTION PROTOTYPES ********************************************/
/* General */
void systick_init(void);
void systick_deinit(void);
void systick_reset(void);
/* Get time */
uint32_t systick_get_ticks(void);
uint16_t systick_get_msec(void);
uint8_t systick_get_sec(void);
uint16_t systick_get_min(void);
/* Callbacks */
void systick_set_callback_msec(void (*func)());
void systick_set_callback_sec(void (*func)());
void systick_set_callback_min(void (*func)());
void systick_clear_callback_msec(void);
void systick_clear_callback_sec(void);
void systick_clear_callback_min(void);


#endif // _ASNX_SYSTICK_H_
