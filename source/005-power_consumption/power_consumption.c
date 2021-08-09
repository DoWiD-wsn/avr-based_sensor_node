/*!
 * @brief   Analysis of the ASN(x) power consumption
 *
 * Application to analyze the ASN(x) power consumption in different
 * modes of operation, namely:
 * 1.) full on
 * 2.) diagnostics deactivated
 * 3.) XBee radio sleeping
 * 4.) power-down mode
 *
 * @file    /005-power_consumption/power_consumption.c
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


/***** DEFINES ********************************************************/
/* Time to be in each mode [s] */
#define TIME_IN_MODE                10


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
    /* 1.) FULL ON */
    _delay_ms(TIME_IN_MODE);
    
    /* 2.) Deactivate diagnostics */
    diag_disable();
    _delay_ms(TIME_IN_MODE);
    
    /* 3.) Put XBee to sleep */
    xbee_sleep_enable();
    _delay_ms(TIME_IN_MODE);
    
    /* 4.) Put MCU to power-down mode */
    sleep_enable();
    sleep_bod_disable();
    sleep_cpu();

    return 0;
}