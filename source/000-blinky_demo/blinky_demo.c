/****
 * @brief   Blinky Demo Application
 *
 * Simple blinky application with the user LEDs.
 *
 * @file    /000-blinky_demo/blinky_demo.c
 * @author  Dominik Widhalm
 * @version 1.2
 * @date    2022/01/31
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

    return 0;
}
