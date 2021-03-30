/**
 *  Header file for ARDUINO LED functionality.
 */

#ifndef _ARDUINO_LED_H_
#define _ARDUINO_LED_H_

/***** INCLUDES ***************************************************************/
/* STD */
#include <stdint.h>
/* AVR */
#include <avr/io.h>


/***** MACROS *****************************************************************/
#define LED_DDR                             DDRB
#define LED_PIN                             PINB
#define LED_PORT                            PORTB
#define LED_GPIO                            PB5


/***** FUNCTION PROTOTYPES ****************************************************/
void led_init(void);
void led_low(void);
void led_high(void);
void led_toggle(void);


#endif // _ARDUINO_LED_H_
