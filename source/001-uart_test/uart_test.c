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
/*** ASNX LIB ***/
#include "adc/adc.h"
#include "uart/uart.h"
#include "util/printf.h"


/***** DEFINES ********************************************************/


/***** GLOBAL VARIABLES ***********************************************/


/***** LOCAL FUNCTION PROTOTYPES **************************************/


/***** FUNCTIONS ******************************************************/


/***** MAIN ROUTINE ***************************************************/
int main(void) {
    /*** Variables ***/
    
    /*** Initialize the hardware ***/
    adc_init(ADC_ADPS_16,ADC_REFS_VCC); // Initialize the ADC
    uart1_init();                       // Initialize the UART0
    printf_init(uart1_putc);            // Initialize the printf function
    
    /* Main Routine */
    while (1) {
        printf("ADC0 value = %d\n", adc_read_input(ADC_CH0));
        printf("ADC1 value = %d\n", adc_read_input(ADC_CH1));
        printf("ADC2 value = %d\n", adc_read_input(ADC_CH2));
        printf("ADC3 value = %d\n", adc_read_input(ADC_CH3));
        printf("\n");
        _delay_ms(1000);
    }

    return(0);
}
