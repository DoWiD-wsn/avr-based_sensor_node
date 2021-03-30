/**
 *  Source file for AVR hardware (HW) functionality.
 */

/***** INCLUDES ***************************************************************/
/* AVR */
#include <avr/io.h>
/* OWN */
#include "hw.h"


/***** LOCAL FUNCTION PROTOTYPES **********************************************/
void hw_set_dir(hw_io_t* gpio, uint8_t dir);
void hw_set_output_state(hw_io_t* gpio, uint8_t state);


/***** FUNCTIONS **************************************************************/
/*
 * Get a structure with the register values of a given GPIO
 */
void hw_get_io(hw_io_t* gpio, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin) {
    gpio->ddr = ddr;
    gpio->port = port;
    gpio->pin = pin;
    gpio->portpin = portpin;
}


/*
 * Set the direction register (DDRx) of a given GPIO
 */
void hw_set_dir(hw_io_t* gpio, uint8_t dir) {
    /* Check if GPIO should be set to input (default) or output */
    (dir == HW_DIR_OUTPUT) ? HW_GPIO_OUTPUT(gpio) : HW_GPIO_INPUT(gpio);
}


/*
 * Set the GPIO to input direction
 */
void hw_set_input(hw_io_t* gpio) {
    /* Set GPIO to input */
    hw_set_dir(gpio, HW_DIR_INPUT);
}


/*
 * Set the GPIO to output direction
 */
void hw_set_output(hw_io_t* gpio) {
    /* Set GPIO to output */
    hw_set_dir(gpio, HW_DIR_OUTPUT);
}


/*
 * Set the output state (PORTx) of a given GPIO
 */
void hw_set_output_state(hw_io_t* gpio, uint8_t state) {
    /* Check if GPIO should be set to low (default) or high */
    (state == HW_STATE_HIGH) ? HW_GPIO_HIGH(gpio) : HW_GPIO_LOW(gpio);
}


/*
 * Set the GPIO output to low state
 */
void hw_set_output_low(hw_io_t* gpio) {
    /* Set GPIO to low level */
    hw_set_output_state(gpio, HW_STATE_LOW);
}


/*
 * Set the GPIO output to high state
 */
void hw_set_output_high(hw_io_t* gpio) {
    /* Set GPIO to high level */
    hw_set_output_state(gpio, HW_STATE_HIGH);
}


/*
 * Toggle the GPIO output state
 */
void hw_set_output_toggle(hw_io_t* gpio) {
    /* Toggle the GPIO state */
    if(hw_read_input(gpio) == HW_STATE_HIGH) {
        hw_set_output_state(gpio, HW_STATE_LOW);
    } else {
        hw_set_output_state(gpio, HW_STATE_HIGH);
    }
}


/*
 * Read the GPIO input state
 */
uint8_t hw_read_input(hw_io_t* gpio) {
    /* Return the state of the GPIO */
    return HW_GPIO_READ(gpio) ? HW_STATE_HIGH : HW_STATE_LOW;
}
