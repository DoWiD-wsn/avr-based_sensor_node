/*!
 * @brief   ASN(x) just idling with everything on
 *
 * Application to analyze the ASN(x) power consumption in idling mode.
 *
 * @file    /006-idling/idling.c
 * @author  Dominik Widhalm
 * @version 1.0.0
 * @date    2021/08/09
 */


/***** INCLUDES *******************************************************/
/*** AVR ***/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
/*** ASNX LIB ***/
/* MCU */
#include "hw/led.h"
#include "i2c/i2c.h"
#include "uart/uart.h"
/* Radio */
#include "xbee/xbee.h"
/* RTC */
#include "rtc/pcf85263.h"
/* Sensors */
#include "sensors/tmp275.h"
/* Util */
#include "util/diagnostics.h"


/***** LOCAL FUNCTIONS ************************************************/
/*!
 * Put a MCUSR register dump into the .noinit section.
 * @see https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
 */
uint8_t MCUSR_dump __attribute__ ((section (".noinit")));
/*!
 * Turn off the WDT as early in the startup process as possible.
 * @see https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
 */
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void get_mcusr(void) {
  MCUSR_dump = MCUSR;
  MCUSR = 0;
  wdt_disable();
}


/*!
 * Sleep function with seconds delay.
 *
 * @param[in]   time    Sleep time in seconds
 */
void _delay_s(uint8_t time) {
    volatile uint8_t cnt;
    for(cnt=0; cnt<time; cnt++) {
        _delay_ms(1000);
    }
}


/*!
 * Main function of the demo application.
 */
int main(void) {
    /*** Local variables ***/
    TMP275_t tmp275;                /* TMP275 sensor device structure */
    
    /* Disable unused hardware modules */
    PRR0 = _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRSPI) | _BV(PRTIM1) | _BV(PRUSART1);
    PRR1 = _BV(PRTIM3);
    
    /* Configure the sleep mode */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    /* Initialize I2C master interface */
    i2c_init();
    /* Initialize Xbee 3 (uses UART0) */
    xbee_init(9600UL);
    /* Wake-up xbee */
    xbee_sleep_disable();
    /* Initialize the diagnostic circuitry */
    diag_init();
    diag_enable();
    /* Initialize the TMP275 sensor */
    tmp275_init(&tmp275, TMP275_I2C_ADDRESS);
    /* Configure the TMP275 sensor (SD; 10-bit mode) */
    tmp275_set_config(&tmp275, 0x21);

    /***** MAIN ROUTINE ***********************************************/
    while(1);

    return 0;
}
