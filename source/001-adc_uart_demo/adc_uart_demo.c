/****
 * @brief   ADC/UART Demo Application
 *
 * Simple demo application to read all single-ended ADC channels and
 * send their last value via UART. Additionally, the MCU's supply
 * voltage is read and transmitted.
 *
 * @file    /001-adc_uart_demo/adc_uart_demo.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.2 $
 * @date    $Date: 2021/08/09 $
 *****/


/***** INCLUDES *******************************************************/
/*** AVR ***/
#include <avr/io.h>
#include <util/delay.h>
/*** ASNX LIB ***/
#include "adc/adc.h"
#include "hw/led.h"
#include "uart/uart.h"
#include "util/printf.h"


/***** MAIN ***********************************************************/
int main(void) {
    /*** Local variables ***/
    float vcc, vbat;
    uint16_t value;
    
    /*** Initialize the hardware ***/
    led_init();                         // Initialize the user LEDs
    led1_low();                         // Initially, set LED1 to low
    led2_high();                        // Initially, set LED2 to high
    adc_init(ADC_ADPS_16,ADC_REFS_VCC); // Initialize the ADC
    uart1_init();                       // Initialize the UART0
    printf_init(uart1_putc);            // Initialize the printf function
    
    /* Main Routine */
    while (1) {
        // Read and print the current supply voltage
        vcc = adc_read_vcc();
        printf("Vcc  = %.2f V\n", vcc);
        /* Supply voltage in volts (V) */
        value = adc_read_input(ADC_CH1);
        /* Calculate voltage from value (voltage divider 1:1) */
        vbat = 2.0 * (value * (vcc / 1023.0));
        printf("Vbat = %.2f V\n", vbat);
        /* Read and print all ADC channels */
        adc_set_reference(ADC_REFS_VCC);
        printf("CH0 = %4d\n", adc_read_input(ADC_CH0));
        printf("CH1 = %4d\n", adc_read_input(ADC_CH1));
        printf("CH2 = %4d\n", adc_read_input(ADC_CH2));
        printf("CH3 = %4d\n", adc_read_input(ADC_CH3));
        printf("CH4 = %4d\n", adc_read_input(ADC_CH4));
        printf("CH5 = %4d\n", adc_read_input(ADC_CH5));
        printf("CH6 = %4d\n", adc_read_input(ADC_CH6));
        printf("CH7 = %4d\n", adc_read_input(ADC_CH7));
        printf("\n");
        /* Update once per second */
        _delay_ms(1000);
    }

    return 0;
}
