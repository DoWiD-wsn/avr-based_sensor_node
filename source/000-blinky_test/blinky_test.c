/**********************************************************************
 *          BLINKY TEST                                               *
 *                                                                    *
 *  Author: Dominik Widhalm                                           *
 *  Date:   2020-06-08                                                *
 *                                                                    *
 *  Brief:                                                            *
 *      Simple test application to let the onboard LED blink.         *
 *                                                                    *
 **********************************************************************/
 

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
/*** AVR ***/
#include <avr/io.h>
#include <util/delay.h>
/*** OWN ***/


/***** DEFINES ********************************************************/
#define LED_DDR     DDRB
#define LED_PIN     PINB
#define LED         PB5


/***** GLOBAL VARIABLES ***********************************************/


/***** LOCAL FUNCTION PROTOTYPES **************************************/


/***** FUNCTIONS ******************************************************/


/***** MAIN ROUTINE ***************************************************/
int main (void) {
    LED_DDR |= _BV(LED);

    while (1) {
        LED_PIN |= _BV(LED);
        _delay_ms(250);
    }

    return(0);
}
