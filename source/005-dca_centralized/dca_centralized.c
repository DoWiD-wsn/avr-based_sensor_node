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
 *
 * @todo    Simplify code and restructure as listed above
 */


/*** APP CONFIGURATION ***/
#define ENABLE_DBG                  (0)     /**< Enable debug output via UART1 (9600 BAUD) */
#define UPDATE_INTERVAL             (1)     /**< Update interval [min] */

/*** Enable (1) or disable (0) sensor measurements ***/
#define ENABLE_DS18B20_T            (1)     /**< Enable the DS18B20 sensor temperature (T) measurement (via OWI) */
#define ENABLE_AM2302_T             (1)     /**< Enable the AM2302 sensor temperature (T) measurement (via OWI) */
#define ENABLE_AM2302_H             (1)     /**< Enable the AM2302 sensor humidity (H) measurement (via OWI) */
#define ENABLE_SHTC3_T              (1)     /**< Enable the SHTC3 sensor temperature (T) measurement (via TWI) */
#define ENABLE_SHTC3_H              (1)     /**< Enable the SHTC3 sensor humidity (H) measurement (via TWI) */

/*! MAC address of the destination */
#define XBEE_DESTINATION_MAC        SEN_MSG_MAC_CH

/*! Maximum number of sensor readings per message */
#define SEN_MSG_NUM_MEASUREMENTS    (14)


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
// 103JT
#include "sensors/jt103.h"
// TMP275
#include "sensors/tmp275.h"
// DS18B20
#if ENABLE_DS18B20_T
#  include "sensors/ds18x20.h"
#endif
// AM2302
#if (ENABLE_AM2302_T || ENABLE_AM2302_H)
#  include "sensors/dht.h"
#endif
// SHTC3
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


/***** GLOBAL VARIABLES ***********************************************/
/*! Previous reset-source indicator */
float rsource_prev = 0.0;


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
 * Activate WDT with shortest delay and wait for reset.
 */
void wait_for_wdt_reset(void) {
    /* Enable Watchdog (time: 15ms) */
    wdt_enable(WDTO_15MS);
    /* Wait for reset */
    while(1);
}


/*!
 * Main function of the demo application.
 */
int main(void) {
    /*** Local variables ***/
    /* Message data structure */
    SEN_MSG_u msg;
    msg.struc.time = 0;
    /* Date/time structure */
    PCF85263_CNTTIME_t time = {0};
    /* Temporary 8-bit register value */
    uint8_t reg = 0;
    /* Temporary sensor measurement variables */
    float vcc = 0.0;
    float measurement = 0.0;
#if (ENABLE_AM2302_T && ENABLE_AM2302_H) || (ENABLE_SHTC3_T && ENABLE_SHTC3_H)
    float measurement2 = 0.0;
#endif
    /* Message index counter */
    uint8_t index = 0;
    uint16_t runtime = 0;
    /* Sensor/measurement-specific variables */
    uint16_t inc_xbee = 0;          /* Incident counter for Xbee functions */
    uint16_t inc_sum = 0;           /* Total incident counter (sum of others) */
    // TMP275
    TMP275_t tmp275;                /* TMP275 sensor device structure */
    uint16_t inc_tmp275 = 0;        /* Incident counter for TMP275 functions */
    uint8_t en_tmp275 = 0;          /* TMP275 is available (1) or has failed (0) */
#if ENABLE_DS18B20_T
    // DS18B20
    DS18X20_t ds18b20;              /* DS18B20 sensor device structure */
    uint16_t inc_ds18b20 = 0;       /* Incident counter for DS18B20 functions */
    uint8_t en_ds18b20 = 0;         /* DS18B20 is available (1) or has failed (0)  */
#endif
#if (ENABLE_AM2302_T || ENABLE_AM2302_H)
    // AM2302
    DHT_t am2302;                   /* AM2302 sensor device structure */
    uint16_t inc_am2302 = 0;        /* Incident counter for AM2302 functions */
    uint8_t en_am2302 = 0;          /* AM2302 is available (1) or has failed (0) */
#endif
#if (ENABLE_SHTC3_T || ENABLE_SHTC3_H)
    // SHTC3
    SHTC3_t shtc3;                  /* SHTC3 sensor device structure */
    uint16_t inc_shtc3 = 0;         /* Incident counter for SHTC3 functions */
    uint8_t en_shtc3 = 0;           /* SHTC3 is available (1) or has failed (0) */
#endif

    /* Disable unused hardware modules */
    PRR0 = _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRSPI);
    /* Configure the sleep mode */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    /* Initialize the user LEDs and disable both by default */
    led_init();
    led1_high();
    led2_high();

    /* Initialize the ADC */
    adc_init(ADC_ADPS_16,ADC_REFS_VCC);

    /* Initialize I2C master interface */
    i2c_init();

#if ENABLE_DBG
    /* Initialize UART1 for debug purposes */
    uart1_init();
    /* Initialize the printf function to use the uart1_putc() function for output */
    printf_init(uart1_putc);
#else
    /* Disable UART1 */
    PRR0 |= _BV(PRUSART1);
#endif

    /* Initialize Xbee 3 (uses UART0) */
    xbee_init(9600UL);

    /* Initialize the diagnostic circuitry */
    diag_init();
    diag_disable();
    
    /* Initialize the fault indicators */
    indicators_init();

    /* Print welcome message */
    printf("=== STARTING UP ... ===\n");

    /* Initialize the RTC */
    if(pcf85263_init() != PCF85263_RET_OK) {
        printf("Couldn't initialize RTC ... aborting!\n");
        wait_for_wdt_reset();
    }
    /* Disable the battery switch */
    if(pcf85263_set_batteryswitch(PCF85263_CTL_BATTERY_BSOFF) != PCF85263_RET_OK) {
        printf("RTC: Battery switch configuration FAILED ... aborting!\n");
        wait_for_wdt_reset();
    }
    /* Disable CLK pin; INTA output */
    if(pcf85263_set_pin_io(PCF85263_CTL_CLKPM | PCF85263_CTL_INTAPM_INTA) != PCF85263_RET_OK) {
        printf("RTC: Battery switch configuration FAILED ... aborting!\n");
        wait_for_wdt_reset();
    }
    /* Enable stop-watch mode (read first to get 100TH and STOPM bits) */
    if(pcf85263_get_function(&reg) != PCF85263_RET_OK) {
        printf("RTC: Function configuration read FAILED ... aborting!\n");
        wait_for_wdt_reset();
    }
    reg |= PCF85263_CTL_FUNC_RTCM;
    if(pcf85263_set_function(reg) != PCF85263_RET_OK) {
        printf("RTC: Function configuration write FAILED ... aborting!\n");
        wait_for_wdt_reset();
    }
    /* Set desired wake-up time */
    time.minutes = UPDATE_INTERVAL;
    if(pcf85263_set_stw_alarm1(&time) != PCF85263_RET_OK) {
        printf("RTC: Alarm time configuration FAILED ... aborting!\n");
        wait_for_wdt_reset();
    }
    /* Enable the alarm */
    if(pcf85263_set_stw_alarm_enables(PCF85263_RTC_ALARM_MIN_A1E) != PCF85263_RET_OK) {
        printf("RTC: Alarm enable configuration FAILED ... aborting!\n");
        wait_for_wdt_reset();
    }
    /* Enable the alarm interrupt */
    if(pcf85263_set_inta_en(PCF85263_CTL_INTA_A1IEA) != PCF85263_RET_OK) {
        printf("RTC: Alarm enable configuration FAILED ... aborting!\n");
        wait_for_wdt_reset();
    }
    /* Start RTC */
    if(pcf85263_start() != PCF85263_RET_OK) {
        printf("Couldn't start RTC ... aborting!\n");
        wait_for_wdt_reset();
    } else {
        printf("... RTC started\n");
    }
    /* Configure INT2 to fire interrupt when logic level is "low" */
    EICRA = 0x00;
    EIMSK = _BV(INT2);

    /* Enable interrupts globally */
    sei();

    /* Initialize the TMP275 sensor */
    if(tmp275_init(&tmp275, TMP275_I2C_ADDRESS) != TMP275_RET_OK) {
        en_tmp275 = 0;
        printf("Couldn't initialize TMP275!\n");
    }
    /* Configure the TMP275 sensor (SD; 10-bit mode) */
    if(tmp275_set_config(&tmp275, 0x21) != TMP275_RET_OK) {
        en_tmp275 = 0;
        printf("Couldn't configure TMP275!\n");
    } else {
        en_tmp275 = 1;
        printf("... TMP275 ready\n");
    }

#if ENABLE_DS18B20_T
    /* Initialize the DS18B20 sensor */
    if(ds18x20_init(&ds18b20, &DDRD, &PORTD, &PIND, PD6) != DS18X20_RET_OK) {
        en_ds18b20 = 0;
        printf("Couldn't initialize DS18B20!\n");
    } else {
        en_ds18b20 = 1;
        printf("... DS18B20 ready\n");
    }
#endif

#if (ENABLE_AM2302_T || ENABLE_AM2302_H)
    /* Initialize the AMS2302 sensor */
    dht_init(&am2302, &DDRD, &PORTD, &PIND, PD7, DHT_DEV_AM2302);
    en_am2302 = 1;
    printf("... AMS2302 ready\n");
#endif

#if (ENABLE_SHTC3_T || ENABLE_SHTC3_H)
    /* Initialize the SHTC3 sensor */
    if(shtc3_init(&shtc3, SHTC3_I2C_ADDRESS) != SHTC3_RET_OK) {
        en_shtc3 = 0;
        printf("Couldn't initialize SHTC3!\n");
    } else {
        en_shtc3 = 1;
        printf("... SHTC3 ready\n");
    }
#endif

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
    printf("\n");

    /***** MAIN ROUTINE ***********************************************/
    while(1) {
        /* Reset sensor message */
        for(index=0; index<SEN_MSG_NUM_MEASUREMENTS; index++) {
            msg.struc.values[index].type = SEN_MSG_TYPE_IGNORE;
            msg.struc.values[index].value = 0;
        }
        index = 0;

        /* Reset timer1 counter value to 0 */
        timer1_set_tcnt(0);
        /* Check if it's a subsequent cycle */
        if(runtime > 0) {
            /* Subsequent cycle -> measurement available */
            msg.struc.values[index].type = SEN_MSG_TYPE_CHK_RUNTIME;
            msg.struc.values[index].value = runtime;
            index++;
#  if ENABLE_DBG
            uint32_t runtime_us = (uint32_t)(runtime) * 256UL;
            printf("... TIMER1 runtime = %d.%03d.%03d us\n",(uint16_t)(runtime_us/1000000UL),(uint16_t)((runtime_us/1000UL)%1000),(uint16_t)(runtime_us%1000));
#  endif
        }
        /* Start timer1 with prescaler 1024 -> measurement interval [256us; 16.78s] */
        timer1_start(TIMER_PRESCALER_1024);

        /* Stop RTC */
        if(pcf85263_stop() != PCF85263_RET_OK) {
            printf("Couldn't stop RTC ... aborting!\n");
            wait_for_wdt_reset();
        }

        /* Enable the self-diagnostics */
        diag_enable();

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

        /* Reset time structure for stop-watch reset below */
        pcf85263_clear_stw_time(&time);
        /* Reset stop-watch time */
        pcf85263_set_stw_time(&time);
        /* Start RTC */
        if(pcf85263_start() != PCF85263_RET_OK) {
            printf("Couldn't re-start RTC ... aborting!\n");
            wait_for_wdt_reset();
        }

        /*** ADC self-diagnosis (via ADC CH0) ***/
        /* Constant voltage divider (1:1) */
        msg.struc.values[index].type = SEN_MSG_TYPE_CHK_ADC;
        msg.struc.values[index].value = diag_adc_check();
        printf("... ADC self-diagnosis: %d\n", msg.struc.values[index].value);
        index++;

        /*** MCU supply voltage (via ADC) ***/
        /* Supply voltage in volts (V) */
        vcc = diag_read_vcc();
        printf("... Supply voltage: %.2f\n", vcc);
        /* Pack measurement into msg as fixed-point number */
        msg.struc.values[index].type = SEN_MSG_TYPE_VSS_MCU;
        msg.struc.values[index].value = fp_float_to_fixed16_10to6(vcc);
        index++;

        /*** Battery voltage (via ADC) ***/
        /* Calculate voltage from value (voltage divider 1:1) */
        measurement = diag_read_vbat();
        printf("... Battery voltage: %.2f\n", measurement);
        /* Pack measurement into msg as fixed-point number */
        msg.struc.values[index].type = SEN_MSG_TYPE_VSS_BAT;
        msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
        index++;

        /*** Xbee3 temperature ***/
        /* Reset the XBee RX buffer */
        xbee_rx_flush();
        /* Temperature in degree Celsius (°C) */
        if(xbee_cmd_get_temperature(&measurement) == XBEE_RET_OK) {
            printf("... Xbee temperature: %.2f\n", measurement);
            /* Pack measurement into msg as fixed-point number */
            msg.struc.values[index].type = SEN_MSG_TYPE_TEMP_RADIO;
            msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
            index++;
            /* Decrement incident counter */
            if(inc_xbee > 0) {
                inc_xbee--;
            }
        } else {
            printf("... Xbee temperature: FAILED!\n");
            /* Increment incident counter */
            if(inc_xbee < X_IC_TH_SINGLE) {
                inc_xbee++;
            } else {
                /* Wait for watchdog reset */
                wait_for_wdt_reset();
            }
        }

        /*** Xbee3 supply voltage ***/
        /* Reset the XBee RX buffer */
        xbee_rx_flush();
        /* Supply voltage in volts (V) */
        if(xbee_cmd_get_vss(&measurement) == XBEE_RET_OK) {
            printf("... Xbee supply voltage: %.2f\n", measurement);
            /* Pack measurement into msg as fixed-point number */
            msg.struc.values[index].type = SEN_MSG_TYPE_VSS_RADIO;
            msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
            index++;
            /* Decrement incident counter */
            if(inc_xbee > 0) {
                inc_xbee--;
            }
        } else {
            printf("... Xbee temperature: FAILED!\n");
            /* Increment incident counter */
            if(inc_xbee < X_IC_TH_SINGLE) {
                inc_xbee++;
            } else {
                /* Wait for watchdog reset */
                wait_for_wdt_reset();
            }
        }

        /*** 103JT thermistor (via ADC CH1) ***/
        /* Temperature in degree Celsius (°C) */
        measurement = jt103_get_temperature(adc_read_input(ADC_CH2));
        printf("... 103JT thermistor: %.2f\n", measurement);
        /* Pack measurement into msg as fixed-point number */
        msg.struc.values[index].type = SEN_MSG_TYPE_TEMP_SURFACE;
        msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
        index++;

        /*** TMP275 ***/
        if(en_tmp275) {
            /* Start a single conversion */
            if(tmp275_set_config(&tmp275, 0xA1) == TMP275_RET_OK) {
                /* Wait for the conversion to finish (55ms) */
                _delay_ms(60);
                /* Get temperature in degree Celsius (°C) */
                if(tmp275_get_temperature(&tmp275, &measurement) == TMP275_RET_OK) {
                    printf("... TMP275 temperature: %.2f\n", measurement);
                    /* Pack measurement into msg as fixed-point number */
                    msg.struc.values[index].type = SEN_MSG_TYPE_TEMP_BOARD;
                    msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
                    index++;
                    /* Decrement incident counter */
                    if(inc_tmp275 > 0) {
                        inc_tmp275--;
                    }
                } else {
                    printf("... TMP275 temperature: FAILED!\n");
                    /* Increment incident counter */
                    if(inc_tmp275 < X_IC_TH_SINGLE) {
                        inc_tmp275++;
                    } else {
                        en_tmp275 = 0;
                    }
                }
            }  else {
                printf("... TMP275 query one-shot (OS): FAILED!\n");
                /* Increment incident counter */
                if(inc_tmp275 < X_IC_TH_SINGLE) {
                    inc_tmp275++;
                } else {
                    en_tmp275 = 0;
                }
            }
        }

#if ENABLE_DS18B20_T
        /*** DS18B20 ***/
        if(en_ds18b20) {
            /* Temperature in degree Celsius (°C) */
            if(ds18x20_get_temperature(&ds18b20, &measurement) == DS18X20_RET_OK) {
                printf("... DS18B20 temperature: %.2f\n", measurement);
                /* Pack measurement into msg as fixed-point number */
                msg.struc.values[index].type = SEN_MSG_TYPE_TEMP_SOIL;
                msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
                index++;
                /* Decrement incident counter */
                if(inc_ds18b20 > 0) {
                    inc_ds18b20--;
                }
            } else {
                printf("... DS18B20 temperature: FAILED!\n");
                /* Increment incident counter */
                if(inc_ds18b20 < X_IC_TH_SINGLE) {
                    inc_ds18b20++;
                } else {
                    en_ds18b20 = 0;
                }
            }
        }
#endif

/* Since AM2302 allows only one result query per 2 seconds, we need to
 * handle three distinct cases: (1) T&H, (2) T, (3) H
 */
#if (ENABLE_AM2302_T && ENABLE_AM2302_H)
        /*** AM2302 (T & H) ***/
        if(en_am2302) {
            /* Temperature in degree Celsius (°C) and relative humidity in percent (% RH) */
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

                /* Decrement incident counter */
                if(inc_am2302 > 0) {
                    inc_am2302--;
                }
            } else {
                printf("... AM2302 temperature & humidity: FAILED!\n");
                /* Increment incident counter */
                if(inc_am2302 < X_IC_TH_SINGLE) {
                    inc_am2302++;
                } else {
                    en_am2302 = 0;
                }
            }
        }
#endif

#if (ENABLE_AM2302_T && !ENABLE_AM2302_H)
        /*** AM2302 (T) ***/
        if(en_am2302) {
            /* Temperature in degree Celsius (°C) */
            if(dht_get_temperature(&am2302, &measurement) == DHT_RET_OK) {
                printf("... AM2302 temperature: %.2f\n", measurement);
                /* Pack measurement into msg as fixed-point number */
                msg.struc.values[index].type = SEN_MSG_TYPE_TEMP_AIR;
                msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
                index++;
                /* Decrement incident counter */
                if(inc_am2302 > 0) {
                    inc_am2302--;
                }
            } else {
                printf("... AM2302 temperature: FAILED!\n");
                /* Increment incident counter */
                if(inc_am2302 < X_IC_TH_SINGLE) {
                    inc_am2302++;
                } else {
                    en_am2302 = 0;
                }
            }
        }
#endif

#if (!ENABLE_AM2302_T && ENABLE_AM2302_H)
        /*** AM2302 (H) ***/
        if(en_am2302) {
            /* Relative humidity in percent (% RH) */
            if(dht_get_humidity(&am2302, &measurement) == DHT_RET_OK) {
                printf("... AM2302 humidity: %.2f\n", measurement);
                /* Pack measurement into msg as fixed-point number */
                msg.struc.values[index].type = SEN_MSG_TYPE_HUMID_AIR;
                msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
                index++;
                /* Decrement incident counter */
                if(inc_am2302 > 0) {
                    inc_am2302--;
                }
            } else {
                printf("... AM2302 humidity: FAILED!\n");
                /* Increment incident counter */
                if(inc_am2302 < X_IC_TH_SINGLE) {
                    inc_am2302++;
                } else {
                    en_am2302 = 0;
                }
            }
        }
#endif

/* For SHTC3, we have three distinct cases, too: (1) T&H, (2) T, (3) H
 */
#if (ENABLE_SHTC3_T && ENABLE_SHTC3_H)
        /*** SHTC3 (T & H) ***/
        if(en_shtc3) {
            /* Temperature in degree Celsius (°C) and relative humidity in percent (% RH) */
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

                /* Decrement incident counter */
                if(inc_shtc3 > 0) {
                    inc_shtc3--;
                }
            } else {
                printf("... SHTC3 temperature & humidity: FAILED!\n");
                /* Increment incident counter */
                if(inc_shtc3 < X_IC_TH_SINGLE) {
                    inc_shtc3++;
                } else {
                    en_shtc3 = 0;
                }
            }
        }
#endif

#if (ENABLE_SHTC3_T && !ENABLE_SHTC3_H)
        /*** SHTC3 (T) ***/
        if(en_shtc3) {
            /* Temperature in degree Celsius (°C) */
            if(shtc3_get_temperature(&shtc3, &measurement) == SHTC3_RET_OK) {
                printf("... SHTC3 temperature: %.2f\n", measurement);
                /* Pack measurement into msg as fixed-point number */
                msg.struc.values[index].type = SEN_MSG_TYPE_TEMP_AIR;
                msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
                index++;
                /* Decrement incident counter */
                if(inc_shtc3 > 0) {
                    inc_shtc3--;
                }
            } else {
                printf("... SHTC3 temperature: FAILED!\n");
                /* Increment incident counter */
                if(inc_shtc3 < X_IC_TH_SINGLE) {
                    inc_shtc3++;
                } else {
                    en_shtc3 = 0;
                }
            }
        }
#endif

#if (!ENABLE_SHTC3_T && ENABLE_SHTC3_H)
        /*** SHTC3 (H) ***/
        if(en_shtc3) {
            /* Relative humidity in percent (% RH) */
            if(shtc3_get_humidity(&shtc3, &measurement) == DHT_RET_OK) {
                printf("... SHTC3 humidity: %.2f\n", measurement);
                /* Pack measurement into msg as fixed-point number */
                msg.struc.values[index].type = SEN_MSG_TYPE_HUMID_AIR;
                msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
                index++;
                /* Decrement incident counter */
                if(inc_shtc3 > 0) {
                    inc_shtc3--;
                }
            } else {
                printf("... SHTC3 humidity: FAILED!\n");
                /* Increment incident counter */
                if(inc_shtc3 < X_IC_TH_SINGLE) {
                    inc_shtc3++;
                } else {
                    en_shtc3 = 0;
                }
            }
        }
#endif

        /*** INCIDENT COUNTER ***/
        inc_sum = inc_xbee;
        inc_sum += inc_tmp275;
#  if ENABLE_DS18B20_T
        inc_sum += inc_ds18b20;
#  endif
#  if (ENABLE_AM2302_T || ENABLE_AM2302_H)
        inc_sum += inc_am2302;
#  endif
#  if (ENABLE_SHTC3_T || ENABLE_SHTC3_H)
        inc_sum += inc_shtc3;
#  endif

        printf("... INCIDENT COUNTER (ENABLE):\n");
        printf("    ... XBEE:    %d\n", inc_xbee);
        printf("    ... TMP275:  %d (%d)\n", inc_tmp275, en_tmp275);
#  if ENABLE_DS18B20_T
        printf("    ... DS18B20: %d (%d)\n", inc_ds18b20, en_ds18b20);
#  endif
#  if (ENABLE_AM2302_T || ENABLE_AM2302_H)
        printf("    ... AM2302:  %d (%d)\n", inc_am2302, en_am2302);
#  endif
#  if (ENABLE_SHTC3_T || ENABLE_SHTC3_H)
        printf("    ... SHTC3:  %d (%d)\n", inc_shtc3, en_shtc3);
#  endif
        printf("    ... TOTAL:   %d\n", inc_sum);
        /* Counter value between 0 and defined threshold */
        msg.struc.values[index].type = SEN_MSG_TYPE_INCIDENTS;
        msg.struc.values[index].value = inc_sum;
        index++;
        /* Check total incident counter */
        if(inc_sum >= X_IC_TH_TOTAL) {
            /* Wait for watchdog reset */
            wait_for_wdt_reset();
        }

        /* Last reboot source */
        msg.struc.values[index].type = SEN_MSG_TYPE_REBOOT;
        /* Update value with MCUSR value (ignoring the JTAG Reset Flag (JTRF)) */
        measurement = x_rst_get_update(MCUSR_dump & 0x0F);
        msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
        index++;
        /* Reset reboot source */
        MCUSR_dump = 0;

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
        /* Send the measurement to the CH */
        int8_t ret = xbee_transmit_unicast(XBEE_DESTINATION_MAC, msg.byte, SEN_MSG_SIZE(index), 0x00);
        if(ret != XBEE_RET_OK) {
            printf("ERROR sending message (%d)!\n",ret);
            /* Increment incident counter */
            if(inc_xbee < X_IC_TH_SINGLE) {
                /* Severe issue, increment by 2 */
                inc_xbee += 2;
            } else {
                /* Wait for watchdog reset */
                wait_for_wdt_reset();
            }
        }
        /* Check if non-blocking UART/XBee mode is used */
        if(XBEE_WRITE_NONBLOCKING == 1) {
            /* Wait until the xbee/uart0 TX buffer is empty */
            retries = 0;
            while(xbee_tx_cnt()>0) {
                /* Check if timeout has been reached */
                if(retries >= ((uint32_t)XBEE_TX_TIMEOUT*1000)) {
                    printf("UART0 TX buffer didn't become empty (%d bytes left) ... aborting!\n",xbee_tx_cnt());
                    /* Wait for watchdog reset */
                    wait_for_wdt_reset();
                } else {
                    /* Wait for some time */
                    retries += XBEE_TX_TIMEOUT_DELAY;
                    _delay_ms(XBEE_TX_TIMEOUT_DELAY);
                }
            }
        }
        /* Increment message number ("time") */
        msg.struc.time++;
        printf("%d sensor values sent! (#%d)\n\n",index,msg.struc.time);

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

        /* Enter power down mode */
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
