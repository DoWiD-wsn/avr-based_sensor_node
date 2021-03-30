/**
 *  Source file for ARDUINO LED functionality.
 */

/***** INCLUDES ***************************************************************/
/* STD */
#include <stddef.h>
/* OWN */
#include "led.h"
#include "hw.h"


/***** GLOBAL VARIABLES *******************************************************/
hw_io_t led = {NULL, NULL, NULL, 0};


/***** FUNCTIONS **************************************************************/
/*
 * Initialize the LED (output & low)
 */
void led_init(void) {
    /* Fill the LED GPIO structure */
    hw_get_io(&led, &LED_DDR, &LED_PORT, &LED_PIN, LED_GPIO);
    /* Set the GPIO to output */
    hw_set_output(&led);
    /* Set the GPIO state to low */
    hw_set_output_low(&led);
}


/*
 * Set the LED level to low
 */
void led_low(void) {
    /* Set the GPIO level of the LED to low */
    hw_set_output_low(&led);
}


/*
 * Set the LED level to high
 */
void led_high(void) {
    /* Set the GPIO level of the LED to high */
    hw_set_output_high(&led);
}


/*
 * Toggle the LED level
 */
void led_toggle(void) {
    /* Toggle the GPIO level of the LED */
    hw_set_output_toggle(&led);
}
