/*!
 * @brief   Demo application for a full sensor node incl. AVR/xbee sleep.
 *
 * Demo application for an environmental monitoring sensor node that
 * periodically measures certain physical quantities and transmits the
 * data to a central cluster head via Zigbee (Xbee). After the sensor
 * value update the Xbee and the AVR are put to sleep and are woken
 * up after a defined period by the onboard RTC. Additionally, the
 * runtime of the active phase can be measure using the MCU's timer1
 * unit. In case of an error that cannot be resolved, the WDT is
 * started and the MCU waits for a WDT reset.
 *
 * @file    /004-sensor_node_demo/sensor_node_demo.c
 * @author  Dominik Widhalm
 * @version 1.5.0
 * @date    2022/01/31
 */


/*** DEMO CONFIGURATION ***/
#define ASNX_VERSION_MINOR          (4)     /**< Used ASN(x) board version minor number (i.e., 0 to 4) */
#define ENABLE_DBG                  (0)     /**< Enable debug output via UART1 (9600 BAUD) */
#define UPDATE_INTERVAL             (10)    /**< Update interval [min] */

/*** Enable (1) or disable (0) measurements/sensors ***/
#define ENABLE_103JT_T              (1)     /**< Enable the 103JT thermistor temperature (T) measurement (via ADC) */
#define ENABLE_TMP275_T             (1)     /**< Enable the TMP275 sensor temperature (T) measurement (via TWI) */
#define ENABLE_DS18B20_T            (1)     /**< Enable the DS18B20 sensor temperature (T) measurement (via OWI) */
#define ENABLE_STEMMA_H             (1)     /**< Enable the STEMMA SOIL sensor humidity (H) measurement (via TWI) */
#define ENABLE_AM2302_T             (1)     /**< Enable the AM2302 sensor temperature (T) measurement (via OWI) */
#define ENABLE_AM2302_H             (1)     /**< Enable the AM2302 sensor humidity (H) measurement (via OWI) */

/*! MAC address of the destination */
#define XBEE_DESTINATION_MAC        SEN_MSG_MAC_CH

/*! Incident counter total threshold */
#define INCIDENT_TOTAL_MAX          (100)
/*! Incident counter single threshold */
#define INCIDENT_SINGLE_MAX         (10)

/*! Maximum number of sensor readings per message */
#define SEN_MSG_NUM_MEASUREMENTS    (14)


/***** INCLUDES *******************************************************/
/*** AVR ***/
#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
/*** ASNX LIB ***/
/* MCU */
#if ENABLE_103JT_T
#include "adc/adc.h"
#endif
#include "hw/led.h"
#include "i2c/i2c.h"
#include "uart/uart.h"
/* Radio */
#include "xbee/xbee.h"
#if ASNX_VERSION_MINOR<1
#include "timer/systick.h"
#else
/* RTC */
#  include "rtc/pcf85263.h"
#endif
/* Sensors */
// 103JT
#if ENABLE_103JT_T
#  include "sensors/jt103.h"
#endif
// TMP275
#if ENABLE_TMP275_T
#  include "sensors/tmp275.h"
#endif
// DS18B20
#if ENABLE_DS18B20_T
#  include "sensors/ds18x20.h"
#endif
// STEMMA
#if ENABLE_STEMMA_H
#  include "sensors/stemma_soil.h"
#endif
// AM2302
#if (ENABLE_AM2302_T || ENABLE_AM2302_H)
#  include "sensors/dht.h"
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
#if ASNX_VERSION_MINOR<1
/*! Variable (flag) for barrier synchronization */
uint8_t barrier = 1;
#endif
/*!
 * Put a MCUSR register dump into the .noinit section.
 * @see https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
 */
uint8_t MCUSR_dump __attribute__ ((section (".noinit")));


/***** LOCAL FUNCTION PROTOTYPES **************************************/
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void sleep_until_reset(uint8_t delay);
#if ASNX_VERSION_MINOR<1
void update(void);
#endif


/***** LOCAL FUNCTIONS ************************************************/
/*!
 * Turn off the WDT as early in the startup process as possible.
 * @see https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
 */
void get_mcusr(void) {
  MCUSR_dump = MCUSR;
  MCUSR = 0;
  wdt_disable();
}


/*!
 * Activate WDT with given delay, put MCU to sleep and wait for reset.
 */
void sleep_until_reset(uint8_t delay) {
    /* Enable Watchdog with given delay */
    wdt_enable(delay);
    /* Put MCU to sleep */
    sleep_enable();
    sleep_bod_disable();
    sleep_cpu();
}


#if ASNX_VERSION_MINOR<1
/***
 * Callback function to be called by the systick timer.
 ***/
void update(void) {
    static uint8_t cnt = 0;
    /* Check if update time has elapsed */
    if(++cnt >= UPDATE_INTERVAL) {
        /* Reset counter */
        cnt = 0;
        /* Set barrier sync flag */
        barrier = 1;
    }
}
#endif


/*!
 * Main function of the demo application.
 */
int main(void) {
    /*** Local variables ***/
    /* Message data structure */
    SEN_MSG_u msg;
    msg.struc.time = 0;
#if ASNX_VERSION_MINOR>=1
    /* Date/time structure */
    PCF85263_CNTTIME_t time = {0};
    /* Temporary variables */
    uint8_t reg = 0;
#endif
#if (ENABLE_103JT_T || ENABLE_TMP275_T || ENABLE_DS18B20_T || ENABLE_STEMMA_H || ENABLE_AM2302_T || ENABLE_AM2302_H)
    /* Temporary variable for sensor measurements */
    float measurement = 0.0;
#endif
#if (ENABLE_AM2302_T && ENABLE_AM2302_H)
    float measurement2 = 0.0;
#endif
    /* Message index counter */
    uint8_t index = 0;
    /* Sensor/measurement-specific variables */
#if ENABLE_TMP275_T
    TMP275_t tmp275;                /* TMP275 sensor device structure */
    uint8_t en_tmp275 = 0;          /* TMP275 is available (1) or has failed (0) */
#endif
#if ENABLE_DS18B20_T
    DS18X20_t ds18b20;              /* DS18B20 sensor device structure */
    uint8_t en_ds18b20 = 0;         /* DS18B20 is available (1) or has failed (0)  */
#endif
#if ENABLE_STEMMA_H
    STEMMA_t stemma;                /* STEMMA sensor device structure */
    uint8_t en_stemma = 0;          /* STEMMA is available (1) or has failed (0) */
    STEMMA_AVG_t avg_stemma = {0};  /* Structure for the STEMMA floating average values */
#endif
#if (ENABLE_AM2302_T || ENABLE_AM2302_H)
    DHT_t am2302;                   /* AM2302 sensor device structure */
    uint8_t en_am2302 = 0;          /* AM2302 is available (1) or has failed (0) */
#endif
    /* Function return value */
    int8_t ret = 0;

    /* Disable unused hardware modules */
#if ASNX_VERSION_MINOR<1
    PRR0 = _BV(PRTIM2) | _BV(PRTIM1) | _BV(PRSPI);
#else
    PRR0 = _BV(PRTIM2) | _BV(PRTIM1) | _BV(PRTIM0) | _BV(PRSPI);
#endif
    PRR1 = _BV(PRTIM3);

#if ASNX_VERSION_MINOR>=1
    /* Configure the sleep mode */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
#endif

    /* Initialize the user LEDs and disable both by default */
    led_init();
    led1_high();
    led2_high();

#if ENABLE_103JT_T
    /* Initialize the ADC */
    adc_init(ADC_ADPS_16,ADC_REFS_VCC);
#else
    /* Disable ADC */
    PRR0 |= _BV(PRADC);
#endif

    /* Initialize I2C master interface */
    i2c_init();

#if ENABLE_DBG
    /* Initialize UART1 for debug purposes */
    uart1_init(9600UL);
    /* Initialize the printf function to use the uart1_write_char() function for output */
    printf_init(uart1_write_char);
#else
    /* Disable UART1 */
    PRR0 |= _BV(PRUSART1);
#endif

    /* Initialize Xbee 3 (uses UART0 @9600 baud; receive via ISR) */
    uart0_init(9600UL);
    uart0_interrupt_enable();
    xbee_init(uart0_write_byte, uart0_pop_byte, uart0_rx_buffer_cnt);

    /* Disable the diagnostic circuitry */
    diag_init();
    diag_disable();

    /* Print welcome message */
    printf("=== STARTING UP ... ===\n");

#if ASNX_VERSION_MINOR<1
    /* Initialize the systick timer */
    systick_init();
    /* Set a systick callback function to be called every second */
    systick_set_callback_min(update);
#else
    /* Initialize the RTC */
    if(pcf85263_init() != PCF85263_RET_OK) {
        printf("Couldn't initialize RTC ... aborting!\n");
        sleep_until_reset(WDTO_15MS);
    }
    /* Disable the battery switch */
    if(pcf85263_set_batteryswitch(PCF85263_CTL_BATTERY_BSOFF) != PCF85263_RET_OK) {
        printf("RTC: Battery switch configuration FAILED ... aborting!\n");
        sleep_until_reset(WDTO_15MS);
    }
    /* Disable CLK pin; INTA output */
    if(pcf85263_set_pin_io(PCF85263_CTL_CLKPM | PCF85263_CTL_INTAPM_INTA) != PCF85263_RET_OK) {
        printf("RTC: Battery switch configuration FAILED ... aborting!\n");
        sleep_until_reset(WDTO_15MS);
    }
    /* Enable stop-watch mode (read first to get 100TH and STOPM bits) */
    if(pcf85263_get_function(&reg) != PCF85263_RET_OK) {
        printf("RTC: Function configuration read FAILED ... aborting!\n");
        sleep_until_reset(WDTO_15MS);
    }
    reg |= PCF85263_CTL_FUNC_RTCM;
    if(pcf85263_set_function(reg) != PCF85263_RET_OK) {
        printf("RTC: Function configuration write FAILED ... aborting!\n");
        sleep_until_reset(WDTO_15MS);
    }
    /* Set desired wake-up time */
    time.minutes = UPDATE_INTERVAL;
    if(pcf85263_set_stw_alarm1(&time) != PCF85263_RET_OK) {
        printf("RTC: Alarm time configuration FAILED ... aborting!\n");
        sleep_until_reset(WDTO_15MS);
    }
    /* Enable the alarm */
    if(pcf85263_set_stw_alarm_enables(PCF85263_RTC_ALARM_MIN_A1E) != PCF85263_RET_OK) {
        printf("RTC: Alarm enable configuration FAILED ... aborting!\n");
        sleep_until_reset(WDTO_15MS);
    }
    /* Enable the alarm interrupt */
    if(pcf85263_set_inta_en(PCF85263_CTL_INTA_A1IEA) != PCF85263_RET_OK) {
        printf("RTC: Alarm enable configuration FAILED ... aborting!\n");
        sleep_until_reset(WDTO_15MS);
    }
    /* Start RTC */
    if(pcf85263_start() != PCF85263_RET_OK) {
        printf("Couldn't start RTC ... aborting!\n");
        sleep_until_reset(WDTO_15MS);
    } else {
        printf("... RTC started\n");
    }
    /* Configure INT2 to fire interrupt when logic level is "low" */
    EICRA = 0x00;
    EIMSK = _BV(INT2);
#endif


    /* Enable interrupts globally */
    sei();

#if ENABLE_TMP275_T
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
#endif

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

#if ENABLE_STEMMA_H
    /* Initialize the STEMMA SOIL sensor */
    if(stemma_init(&stemma, STEMMA_I2C_ADDRESS) != STEMMA_RET_OK) {
        en_stemma = 0;
        printf("Couldn't initialize STEMMA!\n");
    } else {
        en_stemma = 1;
        printf("... STEMMA ready\n");
    }
#endif

#if (ENABLE_AM2302_T || ENABLE_AM2302_H)
    /* Initialize the AMS2302 sensor */
    if(dht_init(&am2302, &DDRD, &PORTD, &PIND, PD7, DHT_DEV_AM2302) != DHT_RET_OK) {
        en_am2302 = 0;
        printf("Couldn't initialize AMS2302!\n");
    } else {
        en_am2302 = 1;
        printf("... AMS2302 ready\n");
    }
#endif

    /* Check Xbee module connection */
    ret = xbee_wait_for_connected();
    if(ret != XBEE_RET_OK) {
        printf("Couldn't connect to the network (%d) ... aborting!\n",ret);
        /* Wait for watchdog reset */
        sleep_until_reset(WDTO_8S);
    }
    /* Print status message */
    printf("... ZIGBEE connected (variable message size; maximum = %d bytes)\n", SEN_MSG_SIZE_MAX);
    printf("\n");

    /***** MAIN ROUTINE ***********************************************/
    while(1) {
        /*** (Re-)enable I2C interface ***/
        /* Reset the TWI */
        i2c_reset();
        
#if ASNX_VERSION_MINOR<1
        /*** Barrier synchronization */
        /* Wait until the barrier sync flag is set */
        while(barrier == 0) {
            _delay_ms(100);
        }
        /* Reset barrier sync flag */
        barrier = 0;
#endif

        /* Reset sensor message */
        for(index=0; index<SEN_MSG_NUM_MEASUREMENTS; index++) {
            msg.struc.values[index].type = SEN_MSG_TYPE_IGNORE;
            msg.struc.values[index].value = 0;
        }
        index = 0;

#if ASNX_VERSION_MINOR>1
        /* Stop RTC */
        if(pcf85263_stop() != PCF85263_RET_OK) {
            printf("Couldn't stop RTC ... aborting!\n");
            sleep_until_reset(WDTO_15MS);
        }
#endif

        /* Wake-up xbee */
        if(xbee_sleep_disable() != XBEE_RET_OK) {
            printf("Couldn't wake-up xbee radio ... aborting!\n");
            /* Wait for watchdog reset */
            sleep_until_reset(WDTO_15MS);
        }


#if ENABLE_103JT_T
        /* Enable ADC */
        adc_enable();
#endif

#if ASNX_VERSION_MINOR>=1
        /* Reset time structure for stop-watch reset below */
        pcf85263_clear_stw_time(&time);
        /* Reset stop-watch time */
        pcf85263_set_stw_time(&time);
        /* Start RTC */
        if(pcf85263_start() != PCF85263_RET_OK) {
            printf("Couldn't start RTC ... aborting!\n");
            sleep_until_reset(WDTO_15MS);
        }
#endif

#if ENABLE_103JT_T
        /*** 103JT thermistor (via ADC CH1) ***/
        /* Temperature in degree Celsius (°C) */
#if ASNX_VERSION_MINOR<1
        measurement = jt103_get_temperature(adc_read_input(ADC_CH1));
#else
        measurement = jt103_get_temperature(adc_read_input(ADC_CH2));
#endif
        printf("... 103JT thermistor: %.2f\n", measurement);
        /* Pack measurement into msg as fixed-point number */
        msg.struc.values[index].type = SEN_MSG_TYPE_TEMP_SURFACE;
        msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
        index++;
#endif

#if ENABLE_TMP275_T
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
                } else {
                    printf("... TMP275 temperature: FAILED!\n");
                }
            }  else {
                printf("... TMP275 query one-shot (OS): FAILED!\n");
            }
        }
#endif

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
            } else {
                printf("... DS18B20 temperature: FAILED!\n");
            }
        }
#endif

#if ENABLE_STEMMA_H
        /*** STEMMA SOIL ***/
        if(en_stemma) {
            /* Relative humidity in percent (% RH) */
            if(stemma_get_humidity_avg(&stemma, &avg_stemma, &measurement) == STEMMA_RET_OK) {
                printf("... STEMMA humidity: %.2f\n", measurement);
                /* Pack measurement into msg as fixed-point number */
                msg.struc.values[index].type = SEN_MSG_TYPE_HUMID_SOIL;
                msg.struc.values[index].value = fp_float_to_fixed16_10to6(measurement);
                index++;
            } else {
                printf("... STEMMA humidity: FAILED!\n");
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
            } else {
                printf("... AM2302 temperature & humidity: FAILED!\n");
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
            } else {
                printf("... AM2302 temperature: FAILED!\n");
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
            } else {
                printf("... AM2302 humidity: FAILED!\n");
            }
        }
#endif

        /* Set the measurements count in data structure */
        msg.struc.cnt = index;
        /* Check Xbee module connection */
        uart0_rx_cb_flush();
        ret = xbee_wait_for_reconnected();
        if(ret != XBEE_RET_OK) {
            printf("ERROR rejoining the netwotk (%d) ... aborting!\n",ret);
            /* Wait for watchdog reset */
            sleep_until_reset(WDTO_8S);
        }
        /* Send the measurement to the CH */
        ret = xbee_transmit_unicast(XBEE_DESTINATION_MAC, msg.byte, SEN_MSG_SIZE(index), 0x00);
        if(ret != XBEE_RET_OK) {
            printf("ERROR sending message (%d)!\n",ret);
        }
        /* Increment message number ("time") */
        msg.struc.time++;
        printf("%d sensor values sent! (#%d)\n\n",index,msg.struc.time);

        /* Send xbee to sleep */
        if(xbee_sleep_enable() != XBEE_RET_OK) {
            printf("Couldn't send xbee radio to sleep ... aborting!\n");
            /* Wait for watchdog reset */
            sleep_until_reset(WDTO_8S);
        }

#if ENABLE_103JT_T
        /* Disable ADC */
        adc_disable();
#endif

#if ASNX_VERSION_MINOR>1
        /* Enter power down mode */
        sleep_enable();
        sleep_bod_disable();
        sleep_cpu();
#endif
    }

    return 0;
}


#if ASNX_VERSION_MINOR>1
/*!
 * INT2 external interrupt 2 interrupt.
 */
ISR(INT2_vect) {
    /* Actually not needed, but still ... */
    sleep_disable();
}
#endif
