/*!
 * @brief   ASN(x) hardware library -- header file
 *
 * Library to support the use of the GPIOs as inputs or outputs.
 *
 * @file    /_asnx_lib_/hw/hw.h
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 *****/

#ifndef _ASNX_HW_H_
#define _ASNX_HW_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>


/***** DEFINES ********************************************************/
/*** Common ***/
#define HW_DIR_INPUT                        (0)
#define HW_DIR_OUTPUT                       (1)
#define HW_STATE_LOW                        (0)
#define HW_STATE_HIGH                       (1)
/*** GPIO access ***/
/*! Set GPIO's direction register to input */
#define HW_GPIO_INPUT(gpio)                 (*(gpio->ddr) &= ~_BV(gpio->portpin))
/*! Set GPIO's direction register to output */
#define HW_GPIO_OUTPUT(gpio)                (*(gpio->ddr) |= _BV(gpio->portpin))
/*! Set GPIO's port register to low */
#define HW_GPIO_LOW(gpio)                   (*(gpio->port) &= ~_BV(gpio->portpin))
/*! Set GPIO's port register to high */
#define HW_GPIO_HIGH(gpio)                  (*(gpio->port) |= _BV(gpio->portpin))
/*! Toggle GPIO's port register */
#define HW_GPIO_TOGGLE(gpio)                (*(gpio->port) ^= _BV(gpio->portpin))
/*! Read GPIO's pin register state */
#define HW_GPIO_READ(gpio)                  (*(gpio->pin) & _BV(gpio->portpin))


/***** STRUCTURES *****************************************************/
/*!
 * A structure to represent a GPIO pin.
 */
typedef struct {
    volatile uint8_t* ddr;      /**< pointer to the GPIO's DDRx register */
    volatile uint8_t* port;     /**< pointer to the GPIO's PORTx register */
    volatile uint8_t* pin;      /**< pointer to the GPIO's PINx register */
    uint8_t portpin;            /**< index of the GPIO pin */
} hw_io_t;


/***** FUNCTION PROTOTYPES ********************************************/
void hw_get_io(hw_io_t* gpio, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin);
void hw_set_input(hw_io_t* gpio);
void hw_set_output(hw_io_t* gpio);
void hw_set_output_low(hw_io_t* gpio);
void hw_set_output_high(hw_io_t* gpio);
void hw_set_output_toggle(hw_io_t* gpio);
uint8_t hw_read_input(hw_io_t* gpio);


#endif // _ASNX_HW_H_
