/****
 * @brief   Demo application for a set of different sensors.
 *
 * Simple demo application to read the measurements of several sensors
 * (e.g., thermistor via ADC, AM2302 and DS18X20 via OWI, and LM75
 * and STEMMA SOIL via TWI) and print the values every 15 seconds
 * (controlled via the systick handler) via UART.
 *
 * @todo    AM2302 not working
 *
 * @file    /002-sensor_demo/sensor_demo.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/14 $
 *****/


/***** INCLUDES *******************************************************/
/*** AVR ***/
#include <avr/io.h>
/*** ASNX LIB ***/
#include "adc/adc.h"
#include "hw/led.h"
#include "sensors/dht.h"
#include "sensors/ds18x20.h"
#include "sensors/jt103.h"
#include "sensors/lm75.h"
#include "sensors/stemma_soil.h"
#include "timer/systick.h"
#include "uart/uart.h"
#include "util/printf.h"


/***** DEFINES ********************************************************/
/* Update interval [s] */
#define UPDATE_INTERVAL     15
/***** GLOBAL VARIABLES ***********************************************/
/* Device handles */
DS18X20_t ds18b20;
DHT_t am2302;


/***** FUNCTION CALLBACK **********************************************/
/***
 * Function to be called by the systick callback to read sensor values
 * and print them via UART.
 ***/
void read_and_print(void) {
    float tmp;
    uint16_t cap;
    printf("=> Internal ADC\n");
    printf("...Supply voltage: %1.2f V\n",adc_read_vcc());
    printf("=> 103JT thermistor\n");
    printf("...Temperature: %2.2f C\n",jt103_get_temperature(adc_read()));
    printf("=> DS18B20\n");
    printf("...Temperature: %2.2f C\n",ds18x20_get_temperature(&ds18b20));
    printf("=> AM2302\n");
    printf("...Humidity: %2.2f %%\n",dht_get_humidity(&am2302));
    printf("...Temperature: %2.2f C\n",dht_get_temperature(&am2302));
    printf("=> LM75\n");
    lm75_get_temperature(&tmp);
    printf("...Temperature: %2.2f C\n",tmp);
    printf("=> STEMMA SOIL\n");
    stemma_get_temperature(&tmp);
    printf("...Temperature: %2.2f C\n",tmp);
    stemma_get_capacity(&cap);
    printf("...Capacity: %4d\n",cap);
    printf("\n");
}


/***
 * Callback function to be called by the systick timer.
 ***/
void update(void) {
    static int cnt = 0;
    /* Toggle LED1 */
    led1_toggle();
    /* Check if 10 seconds have elapsed */
    if(++cnt == UPDATE_INTERVAL) {
        /* Turn LED2 on */
        led2_low();
        /* Call read'n'print function */
        read_and_print();
        /* Turn LED2 off */
        led2_high();
        /* Reset counter */
        cnt = 0;
    }
}


/***** MAIN ***********************************************************/
int main(void) {
    /*** Initialize the hardware ***/
    led_init();                         // Initialize the user LEDs
    led1_low();                         // Initially, set LED1 to low
    led2_high();                        // Initially, set LED2 to high
    adc_init(ADC_ADPS_16,ADC_REFS_VCC); // Initialize the ADC
    lm75_init();                        // Initialize the LM75 sensor
    stemma_init();                      // Initialize the STEMMA SOIL sensor
    ds18x20_init(&ds18b20, &DDRD, &PORTD, &PIND, PD6); // Initialize the DS18B20 sensor
    ds18x20_find(&ds18b20);             // Find the DS18B20 sensor
    dht_init(&am2302, &DDRD, &PORTD, &PIND, PD7, DHT_DEV_AM2302); // Initialize the AMS2302 sensor
    uart1_init();                       // Initialize the UART0
    printf_init(uart1_putc);            // Initialize the printf function
    systick_init();                     // Initialize the systick timer
    
    /* Set a systick callback function to be called every second */
    systick_set_callback_sec(update);
    
    printf("=== STARTING UP ... ===\n");
    
    /* Main routine actually does nothing ...
     * everything happens as callback from the ISR */
    while (1);

    return(0);
}
