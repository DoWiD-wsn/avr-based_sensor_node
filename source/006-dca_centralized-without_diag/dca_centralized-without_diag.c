/*!
 * @file    /005-dca_centralized/dca_centralized-without-diag.c
 * @author  Dominik Widhalm
 * @version 1.0.2
 * @date    2021/11/29
 */


/*** APP CONFIGURATION ***/
#define ENABLE_DBG                  0               /**< Enable debug output via UART1 (9600 BAUD) */
#define UPDATE_INTERVAL             10              /**< Update interval [min] */


/***** INCLUDES *******************************************************/
/*** AVR ***/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
/*** ASNX LIB ***/
/* MCU */
#include "adc/adc.h"
#include "hw/led.h"
#include "i2c/i2c.h"
#include "timer/timer.h"
#include "uart/uart.h"
/* Radio */
#include "xbee/xbee.h"
/* RTC */
#include "rtc/pcf85263.h"
/* Sensors */
#include "sensors/tmp275.h"
#include "sensors/dht.h"
/* Misc */
#include "util/diagnostics.h"
#include "util/fixed_point.h"
#include "util/sensor_msg.h"
#if ENABLE_DBG
#  include "util/printf.h"
#else
#  define printf(...) do { } while (0)
#endif
/* DCA */
#include "dca/indicators.h"


/***** STRUCTURES *****************************************************/
/*!
 * A structure to store the sensor and fault indicator values.
 */
typedef struct {
    /* additional data */
    uint16_t time;          /**< timestamp (2 byte) */
    /* use case data */
    uint16_t t_air;         /**< air temperature (fixed point) */
    uint16_t h_air;         /**< air humidity (fixed point) */
} MSG_t;


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
 * Activate WDT with shortest delay and wait for reset (sort of software-reset).
 */
void wait_for_wdt_reset(void) {
    /* Enable Watchdog (time: 15ms) */
    wdt_enable(WDTO_15MS);
    /* Wait for reset */
    while(1);
}


/*!
 * Debug: print the contents of a given sensor message structure
 */
#if ENABLE_DBG
void dbg_print_msg(MSG_t* msg) {
    printf("\n===== SENSOR MESSAGE CONTENTS =====\n");
    printf("%d message updates\n",msg->time);
    printf("=== SENSOR VALUES ===\n");
    printf("T_air:   %.2f C\n",fp_fixed16_to_float_10to6(msg->t_air));
    printf("T_soil:  %.2f C\n",fp_fixed16_to_float_10to6(msg->t_soil));
    printf("===================================\n\n");
}
#endif


/***** MAIN ***********************************************************/
/*!
 * Main function of the application.
 */
int main(void) {
    /*** Local variables ***/
    /* Message data structure */
    MSG_t msg;
    msg.time = 0;
    /* Date/time structure */
    PCF85263_CNTTIME_t time = {0};
    /* Sensor handles */
    TMP275_t tmp275;                /* TMP275 sensor device structure */
    DHT_t am2302;                   /* AM2302 sensor device structure */
    /* Temporary sensor measurement variables */
    float measurement = 0.0;
    float measurement2 = 0.0;


    /*** 1.) initialize modules ***************************************/
    /* Print welcome message */
    printf("=== STARTING UP ... ===\n");
    
    /* Disable unused hardware modules */
    PRR0 = _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRSPI);
    /* Configure the sleep mode */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    
#if ENABLE_DBG
    /* Initialize UART1 for debug purposes */
    uart1_init();
    /* Initialize the printf function to use the uart1_putc() function for output */
    printf_init(uart1_putc);
#else
    /* Disable UART1 */
    PRR0 |= _BV(PRUSART1);
#endif
    
    /* Initialize the user LEDs and disable both by default */
    led_init();
    /* Initialize the ADC */
    adc_init(ADC_ADPS_16,ADC_REFS_VCC);
    /* Initialize I2C master interface */
    i2c_init();
    /* Initialize Xbee 3 (uses UART0) */
    xbee_init(9600UL);
    
    /* Initialize the diagnostic circuitry */
    diag_init();
    diag_disable();

    /* Initialize the RTC */
    time.minutes = UPDATE_INTERVAL;
    if(pcf85263_init_wakeup_src(&time) != PCF85263_RET_OK) {
        printf("Couldn't initialize RTC ... aborting!\n");
        wait_for_wdt_reset();
    }
    /* Configure INT2 to fire interrupt when logic level is "low" */
    EICRA = 0x00;
    EIMSK = _BV(INT2);

    /* Enable interrupts globally */
    sei();

    /* Initialize the TMP275 sensor */
    if(tmp275_init(&tmp275, TMP275_I2C_ADDRESS) != TMP275_RET_OK) {
        printf("Couldn't initialize TMP275!\n");
        wait_for_wdt_reset();
    }
    /* Configure the TMP275 sensor (SD; 10-bit mode) */
    if(tmp275_set_config(&tmp275, 0x21) != TMP275_RET_OK) {
        printf("Couldn't configure TMP275!\n");
        wait_for_wdt_reset();
    }

    /* Initialize the AMS2302 sensor */
    dht_init(&am2302, &DDRD, &PORTD, &PIND, PD7, DHT_DEV_AM2302);
    printf("... AMS2302 ready\n");

    
    /*** 2.) connect to the Zigbee network ****************************/
    /* Reset the XBee RX buffer */
    xbee_rx_flush();
    /* Check Xbee module connection */
    uint32_t retries = 0;
    /* Check Xbee module connection */
    while(xbee_is_connected() != XBEE_RET_OK) {
        /* Check if timeout [s] has been reached (counter in [ms]) */
        if(retries >= ((uint32_t)XBEE_JOIN_TIMEOUT*1000)) {
            printf("Couldn't connect to the network ... aborting!\n");
            /* Wait for watchdog reset */
            wait_for_wdt_reset();
        } else {
            /* Wait for some time */
            retries += XBEE_JOIN_TIMEOUT_DELAY;
            _delay_ms(XBEE_JOIN_TIMEOUT_DELAY);
        }
    }
    /* Print status message */
    printf("... ZIGBEE connected\n");


    while(1) {
        /*** 3.1) reset RTC (stop-watch mode) *************************/
        /* Stop RTC */
        if(pcf85263_stop() != PCF85263_RET_OK) {
            printf("Couldn't stop RTC ... aborting!\n");
            wait_for_wdt_reset();
        }
        /* Reset time structure for stop-watch reset below */
        pcf85263_clear_stw_time(&time);
        /* Reset stop-watch time */
        pcf85263_set_stw_time(&time);
        /* Start RTC */
        if(pcf85263_start() != PCF85263_RET_OK) {
            printf("Couldn't re-start RTC ... aborting!\n");
            wait_for_wdt_reset();
        }

        /*** 3.2) enable modules/sensors ******************************/
        /* Wake-up xbee */
        if(xbee_sleep_disable() != XBEE_RET_OK) {
            printf("Couldn't wake-up xbee radio ... aborting!\n");
            /* Wait for watchdog reset */
            wait_for_wdt_reset();
        }
        
        /* Reset the TWI */
        i2c_reset();
        /* Enable ADC */
        adc_enable();

        
        /*** 3.3) query sensors ***************************************/
        /* AM2302 - Temperature in degree Celsius (°C) and relative humidity in percent (% RH) */
        if(dht_get_temperature_humidity(&am2302, &measurement, &measurement2) == DHT_RET_OK) {
            printf("... AM2302 temperature: %.2f\n", measurement);
            printf("... AM2302 humidity: %.2f\n", measurement2);
            msg.t_air = fp_float_to_fixed16_10to6(measurement);
            msg.h_air = fp_float_to_fixed16_10to6(measurement2);
            x_ic_dec(X_IC_DEC_NORM);
        } else {
            msg.t_air = 0;
            msg.h_air = 0;
            x_ic_inc(X_IC_INC_NORM);
        }

        /*** 3.4) perform self-diagnostics ****************************/
        /* removed */


        /*** 3.5) send values via Zigbee ******************************/
#if ENABLE_DBG
        /* Print the contents of the message to be sent */
        dbg_print_msg(&msg);
#endif
        /* Send the measurement to the CH */
        int8_t ret = xbee_transmit_unicast(SEN_MSG_MAC_CH, (uint8_t*)&msg, sizeof(MSG_t), 0x00);
        if(ret == XBEE_RET_OK) {
            printf("%d. sensor value update sent!\n\n",msg.time);
            x_ic_dec(X_IC_DEC_NORM);
        } else {
            printf("ERROR sending message (%d)!\n",ret);
            x_ic_inc(X_IC_INC_SERIOUS);
        }
        /* Increment message number ("time") */
        msg.time++;


        /*** 3.6) disable modules/sensors *****************************/
        /* Send xbee to sleep */
        if(xbee_sleep_enable() != XBEE_RET_OK) {
            printf("Couldn't send xbee radio to sleep ... aborting!\n");
            /* Wait for watchdog reset */
            wait_for_wdt_reset();
        }
        /* Disable ADC */
        adc_disable();

        /*** 3.7) put MCU to sleep ************************************/
        sleep_enable();
        sleep_bod_disable();
        sleep_cpu();
    }

    return 0;
}


/*!
 * INT2 external interrupt 2 interrupt.
 */
ISR(INT2_vect) {
    /* Actually not needed, but still ... */
    sleep_disable();
    /* Wait some time to fully wake up */
    _delay_ms(5);
}
