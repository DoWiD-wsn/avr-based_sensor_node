/****
 * @brief   Blinky Demo Application
 *
 * Simple blinky application with the user LEDs.
 *
 * @file    /000-blinky_demo/blinky_demo.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/13 $
 *****/


/***** INCLUDES *******************************************************/
/*** AVR ***/
#include <avr/io.h>
#include <util/delay.h>
/*** ASNX LIB ***/
#include "hw/led.h"


/***** MAIN ***********************************************************/
int main (void) {
    /* Initialize the user LEDs */
    led_init();
    /* Start with LED1 off and LED2 on */
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
