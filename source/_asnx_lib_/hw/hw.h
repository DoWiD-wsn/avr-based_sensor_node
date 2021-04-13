/**
 *  Header file for ASN(x) hardware (HW) functionality.
 */

#ifndef _ASNX_HW_H_
#define _ASNX_HW_H_

/***** INCLUDES ***************************************************************/
#include <stdint.h>


/***** MACROS *****************************************************************/
/*** Common ***/
#define HW_DIR_INPUT                        (0)
#define HW_DIR_OUTPUT                       (1)
#define HW_STATE_LOW                        (0)
#define HW_STATE_HIGH                       (1)
/*** GPIO access ***/
#define HW_GPIO_INPUT(gpio)                 (*(gpio->ddr) &= ~_BV(gpio->portpin))
#define HW_GPIO_OUTPUT(gpio)                (*(gpio->ddr) |= _BV(gpio->portpin))
#define HW_GPIO_LOW(gpio)                   (*(gpio->port) &= ~_BV(gpio->portpin))
#define HW_GPIO_HIGH(gpio)                  (*(gpio->port) |= _BV(gpio->portpin))
#define HW_GPIO_TOGGLE(gpio)                (*(gpio->port) ^= _BV(gpio->portpin))
#define HW_GPIO_READ(gpio)                  (*(gpio->pin) & _BV(gpio->portpin))


/***** STRUCTURES *************************************************************/
typedef struct {
    volatile uint8_t* ddr;
    volatile uint8_t* port;
    volatile uint8_t* pin;
    uint8_t portpin;
} hw_io_t;


/***** FUNCTION PROTOTYPES ****************************************************/
void hw_get_io(hw_io_t* gpio, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin);
void hw_set_input(hw_io_t* gpio);
void hw_set_output(hw_io_t* gpio);
void hw_set_output_low(hw_io_t* gpio);
void hw_set_output_high(hw_io_t* gpio);
void hw_set_output_toggle(hw_io_t* gpio);
uint8_t hw_read_input(hw_io_t* gpio);


#endif // _ASNX_HW_H_
