/****
 * @brief   Demo application for a set of different sensors.
 *
 * Simple demo application to read the measurements of several sensors
 * (e.g., thermistor via ADC, AM2302 and DS18X20 via OWI, and LM75
 * and STEMMA SOIL via TWI) and print the values every 15 seconds
 * (controlled via the systick handler) via UART.
 *
 * @file    /003-xbee_demo/xbee_demo.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1 $
 * @date    $Date: 2021/04/16 $
 *****/


/***** INCLUDES *******************************************************/
/*** AVR ***/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
/*** ASNX LIB ***/
#include "adc/adc.h"
#include "hw/led.h"
#include "uart/uart.h"
#include "util/printf.h"
#include "xbee/xbee.h"
#include "util/sensor_msg.h"


/***** MAIN ***********************************************************/
int main(void) {
    /*** Initialize the hardware ***/
    /* Initialize the user LEDs (LED1 to low and LED2 to high) */
    led_init();
    led1_low();
    led2_high();
    
    /* Initialize the UART0 */
    uart1_init();
    /* Initialize the printf function to use the uart1_putc() function for output */
    printf_init(uart1_putc);
    
    /* Xbee 3 */
    xbee_init(9600UL);
    
    /* Initialize the ADC */
    adc_init(ADC_ADPS_16,ADC_REFS_VCC);
    adc_set_input(ADC_CH1);
    
    /* Print welcome message */
    printf("=== STARTING UP ... ===\n");
    
    /* Check Xbee module connection */
    uint64_t temp;
    uint32_t timeout = 0;
    uint8_t ret;
    do {
        /* Check current connection status */
        ret = xbee_at_local_cmd_read("AI", &temp, 0x01);
        if((ret != XBEE_RET_OK) || (temp != XBEE_AI_RET_SUCCESS)) {
            /* Not connected (yet) -- increment timeout */
            timeout += 500;
            /* Check if timeout has been reached */
            if(timeout >= 10000UL) {
                /* Can't connect to xbee */
                led1_low();
                led2_low();
                /* Endless loop to wait for manual reset */
                while(1);
            }
        }
        /* Wait for some time */
        _delay_ms(500);
    }while((ret != XBEE_RET_OK) && (temp != XBEE_AI_RET_SUCCESS));
    
    /* Print status message */
    printf("... Connected\n");
    
    /* Main routine ... */
    uint8_t cnt = 0;
    while (1) {
        /* Send the measurement to the CH */
        ret = xbee_transmit_unicast(SEN_MSG_MAC_RPI, &cnt, 1, 0x00);
        if(ret == XBEE_RET_OK) {
            printf("Message sent!\n");
            /* Check the transmit response */
            uint8_t status;
            ret = xbee_transmit_status(&status);
            if(ret == XBEE_RET_OK) {
                if(ret == XBEE_TRANSMIT_STAT_DEL_OK) {
                    printf("Positive response received!\n");
                    cnt++;
                } else {
                    printf("NEGATIVE response received (%d)!\n",status);
                }
            } else {
                printf("ERROR receiving response (%d)!\n",ret);
            }
        } else {
            printf("ERROR sending message (%d)!\n",ret);
        }
        printf("\n");
        _delay_ms(2000UL);
    }

    return(0);
}
