/****
 * @brief   Demo application for the Xbee network functionality.
 *
 * Simple demo application to periodically send a value to a specific
 * network destination and increment the value after each cycle.
 *
 * @file    /003-xbee_demo/xbee_demo.c
 * @author  Dominik Widhalm
 * @version 1.2
 * @date    2022/01/31
 *****/


/***** INCLUDES *******************************************************/
/*** AVR ***/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
/*** ASNX LIB ***/
#include "hw/led.h"
#include "uart/uart.h"
#include "util/printf.h"
#include "xbee/xbee.h"


/***** DEFINES ********************************************************/
#define XBEE_DESTINATION_MAC                (0x0013A20041B9FD07)

/***** MAIN ***********************************************************/
int main(void) {
    /*** Initialize the hardware ***/
    /* Initialize the user LEDs (LED1 to low and LED2 to high) */
    led_init();
    led1_low();
    led2_high();

    /* Initialize the UART0 */
    uart1_init(9600UL);
    /* Initialize the printf function to use the uart1_write_char() function for output */
    printf_init(uart1_write_char);
    
    /* Initialize Xbee 3 (uses UART0 @9600 baud; receive via ISR) */
    uart0_init(9600UL);
    uart0_interrupt_enable();
    xbee_init(uart0_write_byte, uart0_pop_byte, uart0_rx_buffer_cnt);
    
    /* Print welcome message */
    printf("=== STARTING UP ... ===\n");
    
    /* Check Xbee module connection */
    if(xbee_wait_for_connected(XBEE_JOIN_TIMEOUT) != XBEE_RET_OK) {
        printf("Couldn't connect to the network ... aborting!\n");
        while(1);
    }
    
    /* Print status message */
    printf("... Connected\n");
    
    /* Main routine ... */
    uint8_t cnt = 0;
    while (1) {
        /* Toggle LED1 */
        led1_toggle();
        /* Send the measurement to the CH */
        int8_t ret = xbee_transmit_unicast(XBEE_DESTINATION_MAC, &cnt, 1, 0x00);
        if(ret == XBEE_RET_OK) {
            printf("Message sent!\n");
            /* Check the transmit response */
            uint8_t status;
            ret = xbee_transmit_status(&status);
            if(ret == XBEE_RET_OK) {
                if(ret == XBEE_TRANSMIT_STAT_DEL_OK) {
                    printf("Positive response received!\n");
                    /* Turn on LED2 */
                    led2_low();
                } else {
                    printf("NEGATIVE response received (%d)!\n",status);
                    /* Turn off LED2 */
                    led2_high();
                }
            } else {
                printf("ERROR receiving response (%d)!\n",ret);
                /* Turn off LED2 */
                led2_high();
            }
        } else {
            printf("ERROR sending message (%d)!\n",ret);
            /* Turn off LED2 */
            led2_high();
        }
        printf("\n");
        cnt++;
        _delay_ms(2000UL);
    }

    return 0;
}
