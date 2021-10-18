/*!
 * @brief   Implementation of a centralized DCA for fault detection
 *
 * Implementation of the dendritic cell algorithm (DCA) in a centralized
 * manner to detect faults. The DCA itself runs on the sink node, but
 * the detection is augmented by the node-level fault indicators that
 * are reported by every sensor node in the network.
 *
 * The procedure is as follows:
 * 1.)  initialize modules
 * 2.)  connect to the Zigbee network
 * 3.1) reset RTC (stop-watch mode)  <--+
 * 3.2) enable modules/sensors          |
 * 3.3) query sensors                   |
 * 3.4) perform self-diagnostics        |
 * 3.5) send values via Zigbee          |
 * 3.6) disable modules/sensors         |
 * 3.7) put MCU to sleep                |
 *  +-----------------------------------+
 *
 * @file    /005-dca_centralized/dca_centralized.c
 * @author  Dominik Widhalm
 * @version 0.1.0
 * @date    2021/10/18
 */


/*** APP CONFIGURATION ***/
#define ENABLE_DBG                  1               /**< Enable debug output via UART1 (9600 BAUD) */
#define UPDATE_INTERVAL             1               /**< Update interval [min] */
#define XBEE_DESTINATION_MAC        SEN_MSG_MAC_CH  /**< MAC address of the destination */
/* Enable (1) or disable (0) sensor measurements */
#define ENABLE_DS18B20_T            1               /**< DS18B20 sensor temperature (T) */
#define ENABLE_AM2302_T             1               /**< AM2302 sensor temperature (T) */
#define ENABLE_AM2302_H             1               /**< AM2302 sensor humidity (H) */
#define ENABLE_SHTC3_T              1               /**< SHTC3 sensor temperature (T) */
#define ENABLE_SHTC3_H              1               /**< SHTC3 sensor humidity (H) */


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
#if ENABLE_DS18B20_T
#  include "sensors/ds18x20.h"
#endif
#if (ENABLE_AM2302_T || ENABLE_AM2302_H)
#  include "sensors/dht.h"
#endif
#if (ENABLE_SHTC3_T || ENABLE_SHTC3_H)
#  include "sensors/shtc3.h"
#endif
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
void dbg_print_msg(SEN_MSG_u* msg) {
    printf("===== SENSOR MESSAGE CONTENTS =====\n");
    printf("%05d. message update\n",msg->struc.time);
    printf("   %2d sensor values\n",msg->struc.cnt);
    printf("=== SENSOR VALUES ===\n");
    for(uint8_t i=0; i<msg->struc.cnt; i++) {
        printf("Value: %5d | Type: %02X\n\n",msg->struc.values[i].value,msg->struc.values[i].type);
    }
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
    SEN_MSG_u msg;
    msg.struc.time = 0;
    /* Date/time structure */
    PCF85263_CNTTIME_t time = {0};
    /* Sensor handles */
    TMP275_t tmp275;                /* TMP275 sensor device structure */
#if ENABLE_DS18B20_T
    DS18X20_t ds18b20;              /* DS18B20 sensor device structure */
#endif
#if (ENABLE_AM2302_T || ENABLE_AM2302_H)
    DHT_t am2302;                   /* AM2302 sensor device structure */
#endif
#if (ENABLE_SHTC3_T || ENABLE_SHTC3_H)
    SHTC3_t shtc3;                  /* SHTC3 sensor device structure */
#endif
    /* Temporary sensor measurement variables */
    float measurement = 0.0;
#if (ENABLE_AM2302_T && ENABLE_AM2302_H) || (ENABLE_SHTC3_T && ENABLE_SHTC3_H)
    float measurement2 = 0.0;
#endif
    /* Diagnostic values */
    float v_bat, v_mcu, v_trx;
    float t_mcu, t_trx, t_brd;
    /* Message index counter */
    uint8_t index = 0;
    /* Runtime measurement */
    uint16_t runtime = 0, runtime_ms = 0;


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
    /* Initialize the fault indicators */
    indicators_init();

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

#if ENABLE_DS18B20_T
    /* Initialize the DS18B20 sensor */
    if(ds18x20_init(&ds18b20, &DDRD, &PORTD, &PIND, PD6) != DS18X20_RET_OK) {
        printf("Couldn't initialize DS18B20!\n");
        wait_for_wdt_reset();
    }
#endif

#if (ENABLE_AM2302_T || ENABLE_AM2302_H)
    /* Initialize the AMS2302 sensor */
    dht_init(&am2302, &DDRD, &PORTD, &PIND, PD7, DHT_DEV_AM2302);
    printf("... AMS2302 ready\n");
#endif

#if (ENABLE_SHTC3_T || ENABLE_SHTC3_H)
    /* Initialize the SHTC3 sensor */
    if(shtc3_init(&shtc3, SHTC3_I2C_ADDRESS) != SHTC3_RET_OK) {
        printf("Couldn't initialize SHTC3!\n");
        wait_for_wdt_reset();
    }
#endif

    
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
    printf("... ZIGBEE connected (variable message size; maximum = %d bytes)\n", SEN_MSG_SIZE_MAX);


    while(1) {
        /* Reset sensor message index */
        index = 0;
        
        
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
        
        /* Reset timer1 counter value to 0 */
        timer1_set_tcnt(0);
        /* Start timer1 with prescaler 1024 -> measurement interval [256us; 16.78s] */
        timer1_start(TIMER_PRESCALER_1024);
        
        /* Enable the self-diagnostics */
        diag_enable();
        /* Reset the TWI */
        i2c_reset();
        /* Enable ADC */
        adc_enable();

        
        /*** 3.3) query sensors ***************************************/
#if ENABLE_DS18B20_T
        /* DS18B20 - Temperature in degree Celsius (°C) */
        if(ds18x20_get_temperature(&ds18b20, &measurement) == DS18X20_RET_OK) {
            printf("... DS18B20 temperature: %.2f\n", measurement);
            
            /* Pack measurement into msg as fixed-point number */
            msg.struc.values[index].type = SEN_MSG_TYPE_TEMP_SOIL;
            msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
            index++;
            x_ic_dec(X_IC_DEC_NORM);
        } else {
            x_ic_inc(X_IC_INC_NORM);
        }
#endif

        /* Since AM2302 allows only one result query per 2 seconds, we need to
         * handle three distinct cases: (1) T&H, (2) T, (3) H
         */
// case (1)
#if (ENABLE_AM2302_T && ENABLE_AM2302_H)
        /* AM2302 - Temperature in degree Celsius (°C) and relative humidity in percent (% RH) */
        if(dht_get_temperature_humidity(&am2302, &measurement, &measurement2) == DHT_RET_OK) {
            printf("... AM2302 temperature: %.2f\n", measurement);
            printf("... AM2302 humidity: %.2f\n", measurement2);

            /* Pack measurement into msg as fixed-point number */
            /* Temperature */
            msg.struc.values[index].type = SEN_MSG_TYPE_TEMP_AIR;
            msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
            index++;
            /* Humidity */
            msg.struc.values[index].type = SEN_MSG_TYPE_HUMID_AIR;
            msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement2);
            index++;
            x_ic_dec(X_IC_DEC_NORM);
        } else {
            x_ic_inc(X_IC_INC_NORM);
        }
#endif
// case (2)
#if (ENABLE_AM2302_T && !ENABLE_AM2302_H)
        /* AM2302 - Temperature in degree Celsius (°C) */
        if(dht_get_temperature(&am2302, &measurement) == DHT_RET_OK) {
            printf("... AM2302 temperature: %.2f\n", measurement);
            
            /* Pack measurement into msg as fixed-point number */
            msg.struc.values[index].type = SEN_MSG_TYPE_TEMP_AIR;
            msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
            index++;
            x_ic_dec(X_IC_DEC_NORM);
        } else {
            x_ic_inc(X_IC_INC_NORM);
        }
#endif
// case (3)
#if (!ENABLE_AM2302_T && ENABLE_AM2302_H)
        /* AM2302 - Relative humidity in percent (% RH) */
        if(dht_get_humidity(&am2302, &measurement) == DHT_RET_OK) {
            printf("... AM2302 humidity: %.2f\n", measurement);
            
            /* Pack measurement into msg as fixed-point number */
            msg.struc.values[index].type = SEN_MSG_TYPE_HUMID_AIR;
            msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
            index++;
            x_ic_dec(X_IC_DEC_NORM);
        } else {
            x_ic_inc(X_IC_INC_NORM);
        }
#endif

/* For SHTC3, we have three distinct cases, too: (1) T&H, (2) T, (3) H */
// case (1)
#if (ENABLE_SHTC3_T && ENABLE_SHTC3_H)
        /* SHTC3 - Temperature in degree Celsius (°C) and relative humidity in percent (% RH) */
        if(shtc3_get_temperature_humidity(&shtc3, &measurement, &measurement2, 1) == SHTC3_RET_OK) {
            printf("... SHTC3 temperature: %.2f\n", measurement);
            printf("... SHTC3 humidity: %.2f\n", measurement2);

            /* Pack measurement into msg as fixed-point number */
            /* Temperature */
            msg.struc.values[index].type = SEN_MSG_TYPE_TEMP_AIR;
            msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
            index++;
            /* Humidity */
            msg.struc.values[index].type = SEN_MSG_TYPE_HUMID_AIR;
            msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement2);
            index++;
            x_ic_dec(X_IC_DEC_NORM);
        } else {
            x_ic_inc(X_IC_INC_NORM);
        }
#endif
// case (2)
#if (ENABLE_SHTC3_T && !ENABLE_SHTC3_H)
        /* SHTC3 - Temperature in degree Celsius (°C) */
        if(shtc3_get_temperature(&shtc3, &measurement) == SHTC3_RET_OK) {
            printf("... SHTC3 temperature: %.2f\n", measurement);
            
            /* Pack measurement into msg as fixed-point number */
            msg.struc.values[index].type = SEN_MSG_TYPE_TEMP_AIR;
            msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
            index++;
            x_ic_dec(X_IC_DEC_NORM);
        } else {
            x_ic_inc(X_IC_INC_NORM);
        }
#endif
// case (3)
#if (!ENABLE_SHTC3_T && ENABLE_SHTC3_H)
        /*** SHTC3 - Relative humidity in percent (% RH) */
        if(shtc3_get_humidity(&shtc3, &measurement) == DHT_RET_OK) {
            printf("... SHTC3 humidity: %.2f\n", measurement);
            
            /* Pack measurement into msg as fixed-point number */
            msg.struc.values[index].type = SEN_MSG_TYPE_HUMID_AIR;
            msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
            index++;
            x_ic_dec(X_IC_DEC_NORM);
        } else {
            x_ic_inc(X_IC_INC_NORM);
        }
#endif


        /*** 3.4) perform self-diagnostics ****************************/
        /* MCU surface temperature (103JT thermistor via ADC CH2) */
        t_mcu = diag_read_tsurface();
        /* Board temperature (TMP275 via TWI) */
        if(tmp275_set_config(&tmp275, 0xA1) == TMP275_RET_OK) {
            /* Wait for the conversion to finish (55ms) */
            _delay_ms(60);
            /* Get temperature in degree Celsius (°C) */
            if(tmp275_get_temperature(&tmp275, &t_brd) == TMP275_RET_OK) {
                x_ic_dec(X_IC_DEC_NORM);
            } else {
                x_ic_inc(X_IC_INC_NORM);
            }
        }  else {
            x_ic_inc(X_IC_INC_NORM);
        }
        /* Xbee3 temperature */
        xbee_rx_flush();
        if(xbee_cmd_get_temperature(&t_trx) == XBEE_RET_OK) {
            x_ic_dec(X_IC_DEC_NORM);
        } else {
            x_ic_inc(X_IC_INC_NORM);
        }
        /* MCU supply voltage in volts (V) */
        v_mcu = diag_read_vcc();
        /* Battery voltage (via ADC) */
        v_bat = diag_read_vbat();
        /* Xbee3 supply voltage */
        xbee_rx_flush();
        if(xbee_cmd_get_vss(&v_trx) == XBEE_RET_OK) {
            x_ic_dec(X_IC_DEC_NORM);
        } else {
            x_ic_inc(X_IC_INC_NORM);
        }
        /* Active runtime measurement */
        if(runtime > 0) {
            /* Subsequent cycle -> measurement available */
            // runtime_us = (uint32_t)(runtime) * 256UL;
            runtime_ms = (uint16_t)(((double)runtime * 256.0) / 1000.0);
        }

        /* Node temperature monitor (X_NT) */
        measurement = x_nt_get_normalized(t_mcu, t_brd, t_trx);
        msg.struc.values[index].type = SEN_MSG_TYPE_X_NT;
        msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
        index++;
        
        /* Supply voltage monitor (X_VS) */
        measurement = x_vs_get_normalized(v_mcu, v_trx);
        msg.struc.values[index].type = SEN_MSG_TYPE_X_VS;
        msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
        index++;
        
        /* Battery voltage monitor (X_BAT) */
        measurement = x_bat_get_normalized(v_bat);
        msg.struc.values[index].type = SEN_MSG_TYPE_X_BAT;
        msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
        index++;
        
        /* Active runtime monitor (X_ART) */
        measurement = x_art_get_normalized(runtime_ms);
        msg.struc.values[index].type = SEN_MSG_TYPE_X_ART;
        msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
        index++;
        
        /* Reset monitor (X_RST) */
        measurement = x_rst_get_normalized(MCUSR_dump & 0x0F);
        MCUSR_dump = 0;
        msg.struc.values[index].type = SEN_MSG_TYPE_X_RST;
        msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
        index++;
        
        /* Software incident counter (X_IC) */
        measurement = x_ic_get_normalized();
        msg.struc.values[index].type = SEN_MSG_TYPE_X_IC;
        msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
        index++;
        
        /* ADC self-check (X_ADC) */
        measurement = x_adc_get_normalized(diag_adc_check());
        msg.struc.values[index].type = SEN_MSG_TYPE_X_ADC;
        msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
        index++;
        
        /* USART self-check (X_USART) */
        measurement = x_usart_get_normalized(NULL, NULL, 0);            // TODO: implement USART check!?
        msg.struc.values[index].type = SEN_MSG_TYPE_X_USART;
        msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
        index++;

        /* Check incident counter value */
        if(x_ic_get() >= X_IC_THRESHOLD) {
            printf("There were too many software incidents ... aborting!\n");
            /* Wait for watchdog reset */
            wait_for_wdt_reset();
        }


        /*** 3.5) send values via Zigbee ******************************/
        /* Reset the XBee RX buffer */
        xbee_rx_flush();
        /* Set the measurements count in data structure */
        msg.struc.cnt = index;
        /* Check Xbee module connection */
        retries = 0;
        while(xbee_is_connected() != XBEE_RET_OK) {
            /* Check if timeout [s] has been reached (counter in [ms]) */
            if(retries >= ((uint32_t)XBEE_JOIN_TIMEOUT*1000)) {
                printf("Couldn't re-connect to the network ... aborting!\n");
                /* Wait for watchdog reset */
                wait_for_wdt_reset();
            } else {
                /* Wait for some time */
                retries += XBEE_JOIN_TIMEOUT_DELAY;
                _delay_ms(XBEE_JOIN_TIMEOUT_DELAY);
            }
        }
#if ENABLE_DBG
        /* Print the contents of the message to be sent */
        dbg_print_msg(&msg);
#endif
        /* Send the measurement to the CH */
        int8_t ret = xbee_transmit_unicast(XBEE_DESTINATION_MAC, msg.byte, SEN_MSG_SIZE(index), 0x00);
        if(ret != XBEE_RET_OK) {
            printf("ERROR sending message (%d)!\n",ret);
            x_ic_inc(X_IC_INC_SERIOUS);
        } else {
            x_ic_dec(X_IC_DEC_NORM);
        }
        /* Increment message number ("time") */
        msg.struc.time++;
        printf("%d sensor values sent! (#%d)\n\n",index,msg.struc.time);


        /*** 3.6) disable modules/sensors *****************************/
        /* Send xbee to sleep */
        if(xbee_sleep_enable() != XBEE_RET_OK) {
            printf("Couldn't send xbee radio to sleep ... aborting!\n");
            /* Wait for watchdog reset */
            wait_for_wdt_reset();
        }
        /* Disable ADC */
        adc_disable();
        /* Disable the self-diagnostics */
        diag_disable();
        /* Stop timer1 to save runtime measurement */
        timer1_stop();
        /* Save timer1 counter value */
        runtime = timer1_get_tcnt();


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
