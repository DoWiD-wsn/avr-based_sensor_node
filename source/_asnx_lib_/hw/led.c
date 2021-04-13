/**
 *  Source file for ASN(x) user LEDs.
 */

/***** INCLUDES ***************************************************************/
/* STD */
#include <stddef.h>
/* OWN */
#include "hw/hw.h"
#include "led.h"


/***** GLOBAL VARIABLES *******************************************************/
hw_io_t led1 = {NULL, NULL, NULL, 0};
hw_io_t led2 = {NULL, NULL, NULL, 0};


/***** FUNCTIONS **************************************************************/
/*
 * Initialize the LEDs (output & low)
 */
void led_init(void) {
    /* Fill the LED GPIO structure */
    hw_get_io(&led1, &LED_DDR, &LED_PORT, &LED_PIN, LED1_GPIO);
    hw_get_io(&led2, &LED_DDR, &LED_PORT, &LED_PIN, LED2_GPIO);
    /* Set the GPIO to output */
    hw_set_output(&led1);
    hw_set_output(&led2);
    /* Set the GPIO state to low */
    hw_set_output_low(&led1);
    hw_set_output_low(&led2);
}


/*
 * Set the LED1 level to low
 */
void led1_low(void) {
    /* Set the GPIO level of the LED1 to low */
    hw_set_output_low(&led1);
}


/*
 * Set the LED2 level to low
 */
void led2_low(void) {
    /* Set the GPIO level of the LED2 to low */
    hw_set_output_low(&led2);
}


/*
 * Set the LED1 level to high
 */
void led1_high(void) {
    /* Set the GPIO level of the LED1 to high */
    hw_set_output_high(&led1);
}


/*
 * Set the LED2 level to high
 */
void led2_high(void) {
    /* Set the GPIO level of the LED2 to high */
    hw_set_output_high(&led2);
}


/*
 * Toggle the LED1 level
 */
void led1_toggle(void) {
    /* Toggle the GPIO level of the LED1 */
    hw_set_output_toggle(&led1);
}


/*
 * Toggle the LED2 level
 */
void led2_toggle(void) {
    /* Toggle the GPIO level of the LED2 */
    hw_set_output_toggle(&led2);
}
