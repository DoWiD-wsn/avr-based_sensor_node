/*!
 * @brief   ASN(x) user LED library -- source file
 *
 * Library to support the use of the ASN(x) user LEDs.
 *
 * @file    /_asnx_lib_/hw/led.c
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 */

/***** INCLUDES *******************************************************/
#include "led.h"
/*** STD ***/
#include <stddef.h>
/*** ASNX LIB ***/
#include "hw/hw.h"


/***** GLOBAL VARIABLES ***********************************************/
/*! GPIO struct for the user LED1 */
hw_io_t led1 = {NULL, NULL, NULL, 0};
/*! GPIO struct for the user LED2 */
hw_io_t led2 = {NULL, NULL, NULL, 0};


/***** FUNCTIONS ******************************************************/
/*!
 * Initialize the user LEDs (set registers accordingly).
 * Initially, both LEDs are deactivated (set to low).
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


/*!
 * Activate user LED1 (set GPIO to low).
 */
void led1_low(void) {
    /* Set the GPIO level of the LED1 to low */
    hw_set_output_low(&led1);
}


/*!
 * Activate user LED2 (set GPIO to low).
 */
void led2_low(void) {
    /* Set the GPIO level of the LED2 to low */
    hw_set_output_low(&led2);
}


/*!
 * Deactivate user LED1 (set GPIO to high).
 */
void led1_high(void) {
    /* Set the GPIO level of the LED1 to high */
    hw_set_output_high(&led1);
}


/*!
 * Deactivate user LED2 (set GPIO to high).
 */
void led2_high(void) {
    /* Set the GPIO level of the LED2 to high */
    hw_set_output_high(&led2);
}


/*!
 * Toggle user LED1 state.
 */
void led1_toggle(void) {
    /* Toggle the GPIO level of the LED1 */
    hw_set_output_toggle(&led1);
}


/*!
 * Toggle user LED2 state.
 */
void led2_toggle(void) {
    /* Toggle the GPIO level of the LED2 */
    hw_set_output_toggle(&led2);
}
