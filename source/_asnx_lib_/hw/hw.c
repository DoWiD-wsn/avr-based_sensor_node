/*!
 * @brief   ASN(x) hardware library -- source file
 *
 * Library to support the use of the GPIOs as inputs or outputs.
 *
 * @file    /_asnx_lib_/hw/hw.c
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 */


/***** INCLUDES *******************************************************/
#include "hw.h"
/*** AVR ***/
#include <avr/io.h>


/***** LOCAL FUNCTION PROTOTYPES **************************************/
void hw_set_dir(hw_io_t* gpio, uint8_t dir);
void hw_set_output_state(hw_io_t* gpio, uint8_t state);


/***** FUNCTIONS ******************************************************/
/*!
 * Get a structure with the register values of a given GPIO.
 *
 * @param[out]  gpio    Pointer to the structure to be filled
 * @param[in]   ddr     Pointer to the GPIO's DDRx register
 * @param[in]   port    Pointer to the GPIO's PORTx register
 * @param[in]   pin     Pointer to the GPIO's PINx register
 * @param[in]   portpin Index of the GPIO pin
 */
void hw_get_io(hw_io_t* gpio, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin) {
    gpio->ddr = ddr;
    gpio->port = port;
    gpio->pin = pin;
    gpio->portpin = portpin;
}


/*!
 * Set the direction register (DDRx) of a given GPIO.
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 * @param[in]   dir     GPIO direction
 */
void hw_set_dir(hw_io_t* gpio, uint8_t dir) {
    /* Check if GPIO should be set to input (default) or output */
    (dir == HW_DIR_OUTPUT) ? HW_GPIO_OUTPUT(gpio) : HW_GPIO_INPUT(gpio);
}


/*!
 * Set a given GPIO to input.
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 */
void hw_set_input(hw_io_t* gpio) {
    /* Set GPIO to input */
    hw_set_dir(gpio, HW_DIR_INPUT);
}


/*!
 * Set a given GPIO to output.
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 */
void hw_set_output(hw_io_t* gpio) {
    /* Set GPIO to output */
    hw_set_dir(gpio, HW_DIR_OUTPUT);
}


/*!
 * Set the output state (PORTx) of a given GPIO.
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 * @param[in]   state   GPIO state (low or high)
 */
void hw_set_output_state(hw_io_t* gpio, uint8_t state) {
    /* Check if GPIO should be set to low (default) or high */
    (state == HW_STATE_HIGH) ? HW_GPIO_HIGH(gpio) : HW_GPIO_LOW(gpio);
}


/*!
 * Set the output state of the GPIO to low ("0").
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 */
void hw_set_output_low(hw_io_t* gpio) {
    /* Set GPIO to low level */
    hw_set_output_state(gpio, HW_STATE_LOW);
}


/*!
 * Set the output state of the GPIO to high ("1").
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 */
void hw_set_output_high(hw_io_t* gpio) {
    /* Set GPIO to high level */
    hw_set_output_state(gpio, HW_STATE_HIGH);
}


/*!
 * Toggle the output state of the GPIO.
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 */
void hw_set_output_toggle(hw_io_t* gpio) {
    /* Toggle the GPIO */
    HW_GPIO_TOGGLE(gpio);
}


/*!
 * Read the GPIO input state (pin).
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 * @return      High (1) in case of high; low (0) otherwise
 */
uint8_t hw_read_input(hw_io_t* gpio) {
    /* Return the state of the GPIO */
    return HW_GPIO_READ(gpio) ? HW_STATE_HIGH : HW_STATE_LOW;
}
