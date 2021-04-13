/**********************************************************************
 *          BLINKY TEST                                               *
 *                                                                    *
 *  Author: Dominik Widhalm                                           *
 *  Date:   2021-04-13                                                *
 *                                                                    *
 *  Brief:                                                            *
 *      Simple blinky application for the user LEDs.                  *
 *                                                                    *
 **********************************************************************/
 

/***** INCLUDES *******************************************************/
/*** AVR ***/
#include <avr/io.h>
#include <util/delay.h>
/*** _asnx_lib_ ***/
#include "hw/led.h"


/***** MAIN ROUTINE ***************************************************/
int main (void) {
    /* Initialize the user LEDs */
    led_init();
    led1_low();
    led2_high();

    /* Toggle them every 500ms */
    while (1) {
        led1_toggle();
        led2_toggle();
        _delay_ms(500);
    }

    return(0);
}
