/**
 *  Header file for AVR SYSTICK functionality.
 * 
 *  The systick functionality uses the timer0 module!
 */

#ifndef _AVR_SYSTICK_H_
#define _AVR_SYSTICK_H_

/***** INCLUDES ***************************************************************/
#include <stdint.h>


/***** MACROS *****************************************************************/


/***** STRUCTURES *************************************************************/
typedef struct {
    uint16_t msec;
    uint8_t sec;
    uint16_t min;
} systick_t;


/***** GLOBAL VARIABLES *******************************************************/
extern systick_t systime;


/***** FUNCTION PROTOTYPES ****************************************************/
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


#endif // _AVR_SYSTICK_H_
