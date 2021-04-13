/**
 *  Header file for ASN(x) user LEDs.
 */

#ifndef _ASNX_LED_H_
#define _ASNX_LED_H_

/***** INCLUDES ***************************************************************/
/* STD */
#include <stdint.h>
/* AVR */
#include <avr/io.h>


/***** MACROS *****************************************************************/
#define LED_DDR                             DDRD
#define LED_PIN                             PIND
#define LED_PORT                            PORTD
/* User LED 1 */
#define LED1_GPIO                           PD5
/* User LED 2 */
#define LED2_GPIO                           PD4


/***** FUNCTION PROTOTYPES ****************************************************/
void led_init(void);
void led1_low(void);
void led2_low(void);
void led1_high(void);
void led2_high(void);
void led1_toggle(void);
void led2_toggle(void);


#endif // _ASNX_LED_H_
