/**********************************************************************
 *          UART TEST                                                 *
 *                                                                    *
 *  Author: Dominik Widhalm                                           *
 *  Date:   2020-07-06                                                *
 *                                                                    *
 *  Brief:                                                            *
 *      Simple test application to verify the UART common library     *
 *      functionality including the sending of an incremental number  *
 *      via UART, whereby the UART is used as a serial interface.     *
 *                                                                    *
 *  Notes:                                                            *
 *      *) Successfully tested UART library and printf ...            *
 *         ... using the USB/UART from Arduino                        *
 *         => Terminate Picocom before flashing the uC                *
 *      *) PICOCOM: $ picocom -b 9600 -c --imap lfcrlf /dev/ttyACM0   *
 *         -) Permission denied:                                      *
 *            $ sudo apt remove modemmanager                          *
 *            $ sudo adduser $USER dialout                            *
 *            => reboot                                               *
 *                                                                    *
 **********************************************************************/


/***** INCLUDES *******************************************************/
/*** AVR ***/
#include <avr/io.h>
#include <util/delay.h>
/*** OWN ***/
#include "uart/uart.h"
#include "util/printf.h"


/***** DEFINES ********************************************************/


/***** GLOBAL VARIABLES ***********************************************/


/***** LOCAL FUNCTION PROTOTYPES **************************************/


/***** FUNCTIONS ******************************************************/


/***** MAIN ROUTINE ***************************************************/
int main(void) {
    /*** Variables ***/
    uint8_t count=0;
    
    /*** Initialize the hardware ***/
    /* PRINT/UART */
    uart1_init();                       // Initialize the UART0
    printf_init(uart1_putc);            // Initialize the printf function
    
    uart1_putc('H');
    uart1_putc('e');
    uart1_putc('l');
    uart1_putc('l');
    uart1_putc('o');
    uart1_putc('!');
    uart1_putc('\n');
    uart1_puts("What's up?\n");
    
    /* Main Routine */
    while (1) {
        printf("Test it! x = %d\n", count++);
        _delay_ms(1000);
    }

    return(0);
}
