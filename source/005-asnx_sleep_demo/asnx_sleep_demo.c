/****
 * @brief   ASN(x) demo application including sleep functionality.
 *
 * Demo application based on the sensor node demo (004), but with
 * Xbee pin-sleep enabled, that is, the xbee radio is only active
 * while updating the values and is put to sleep the remaining time.
 * Also, the PCF85263A RTC is included to have a reliable wake-up
 * trigger for the AVR when entering power-down mode.
 *
 * @file    /005-asnx_sleep_demo/asnx_sleep_demo.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1 $
 * @date    $Date: 2021/05/10 $
 *****/


/*** DEMO CONFIGURATION ***/
/* Enable debug output via UART1 */
#define ENABLE_DBG              (1)

/* Update interval [min] */
#define UPDATE_INTERVAL         (10)

/* MAC address of the destination */
#define XBEE_DESTINATION_MAC    SEN_MSG_MAC_CH

/* Incident counter total threshold */
#define INCIDENT_TOTAL_MAX      (100)
/* Incident counter single threshold */
#define INCIDENT_SINGLE_MAX     (10)

/* Sensor node configuration (0 ... stemma soil, ds18b20 / 1 ... am2302) */
#define NODE_CONFIGURATION      (0)
/* Thermistor surface measurement available (0 ... no / 1 ... yes) */
#define NODE_103JT_AVAILABLE    (1)

/* Measurement message data indices */
#define MSG_VALUE_ADC_SELF      (0)
#define MSG_VALUE_MCU_VSS       (1)
#define MSG_VALUE_BAT_VSS       (2)
#define MSG_VALUE_XBEE_T        (3)
#define MSG_VALUE_XBEE_VSS      (4)
#define MSG_VALUE_103JT_T       (5)
#define MSG_VALUE_TMP275_T      (6)
#define MSG_VALUE_DS18B20_T     (7)
#define MSG_VALUE_STEMMA_H      (8)
#define MSG_VALUE_AM2302_T      (9)
#define MSG_VALUE_AM2302_H      (10)
#define MSG_VALUE_INCIDENT      (11)
#define MSG_VALUE_REBOOOT       (12)

/* Xbee sleep-request pin */
#define XBEE_SLEEP_REQ_DDR      (DDRC)
#define XBEE_SLEEP_REQ_PORT     (PORTC)
#define XBEE_SLEEP_REQ_PIN      (PINC)
#define XBEE_SLEEP_REQ_GPIO     (PC6)
/* Xbee sleep-indicator pin */
#define XBEE_SLEEP_IND_DDR      (DDRC)
#define XBEE_SLEEP_IND_PORT     (PORTC)
#define XBEE_SLEEP_IND_PIN      (PINC)
#define XBEE_SLEEP_IND_GPIO     (PC7)


/***** INCLUDES *******************************************************/
/*** AVR ***/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
/*** ASNX LIB ***/
/* MCU */
#include "adc/adc.h"
#include "hw/led.h"
#include "timer/systick.h"
#include "uart/uart.h"
/* Radio */
#include "xbee/xbee.h"
/* Sensors */
#include "sensors/tmp275.h"
TMP275_t tmp275;
#if NODE_CONFIGURATION==0
#  include "sensors/ds18x20.h"
DS18X20_t ds18b20;
#  include "sensors/stemma_soil.h"
STEMMA_t stemma;
#elif NODE_CONFIGURATION==1
#  include "sensors/dht.h"
DHT_t am2302;
#endif
#if NODE_103JT_AVAILABLE
#  include "sensors/jt103.h"
#endif
/* Misc */
#include "util/fixed_point.h"
#include "util/sensor_msg.h"
#if ENABLE_DBG
#  include "util/printf.h"
#else
#  define printf(...) do { } while (0)
#endif


/***** GLOBAL VARIABLES ***********************************************/
/* Variable (flag) for barrier synchronization */
uint8_t barrier = 1;


/***
 * Turn off the WDT as early in the startup process as possible.
 * @see https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
 ***/
uint8_t MCUSR_dump __attribute__ ((section (".noinit")));
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void get_mcusr(void) {
  MCUSR_dump = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

/***
 * Callback function to be called by the systick timer.
 ***/
void update(void) {
    static int cnt = 0;
    /* Check if update time has elapsed */
    if(++cnt >= UPDATE_INTERVAL) {
        /* Reset counter */
        cnt = 0;
        /* Set barrier sync flag */
        barrier = 1;
    }
}


/***
 * LED SHOW STARTUP
 * asynchronous blinking LED for ~10s
 ***/
void led_show_startup(void) {
    uint8_t cnt;
    led1_low();
    for(cnt=0; cnt<20; cnt++) {
        led2_low();
        _delay_ms(50);
        led2_high();
        _delay_ms(450);
    }
    led1_high();
    led2_high();
}


/***** MAIN ***********************************************************/
int main(void) {
    /*** Local variables ***/
    /* Message data structure */
    SEN_MSG_u msg;
    /* Xbee sleep req & ind GPIO structures */
    hw_io_t xslp_req, xslp_ind;
    /* Temporary variable for sensor measurements */
    float measurement = 0.0, vcc = 0.0;
    uint16_t adcres = 0;
    /* Temporary counter variable */
    uint8_t i = 0;
    /* Incident counter(s) */
    uint16_t inc_xbee = 0;          /**< Incident counter for Xbee functions */
    uint16_t inc_tmp275 = 0;        /**< Incident counter for TMP275 functions */
    uint16_t inc_ds18b20 = 0;       /**< Incident counter for DS18B20 functions */
    uint16_t inc_stemma = 0;        /**< Incident counter for STEMMA SOIL functions */
    uint16_t inc_am2302 = 0;        /**< Incident counter for AM2302 functions */
    uint16_t inc_sum = 0;           /**< Total incident counter (sum of others) */
    /* Sensors working */
    uint8_t en_tmp275 = 0;          /**< TMP275 is available (1) or has failed (0) */
#if NODE_CONFIGURATION==0
    uint8_t en_ds18b20 = 0;         /**< DS18B20 is available (1) or has failed (0)  */
    uint8_t en_stemma = 0;          /**< STEMMA is available (1) or has failed (0) */
    STEMMA_AVG_t avg_stemma = {0};  /**< Structure for the STEMMA floating average values */
#elif NODE_CONFIGURATION==1
    uint8_t en_am2302 = 0;          /**< AM2302 is available (1) or has failed (0) */
#endif
    
    /*** Initialize the xbee sleep req & ind structures ***/
    hw_get_io(&xslp_req, &XBEE_SLEEP_REQ_DDR, &XBEE_SLEEP_REQ_PORT, &XBEE_SLEEP_REQ_PIN, XBEE_SLEEP_REQ_GPIO);
    hw_get_io(&xslp_ind, &XBEE_SLEEP_IND_DDR, &XBEE_SLEEP_IND_PORT, &XBEE_SLEEP_IND_PIN, XBEE_SLEEP_IND_GPIO);
    hw_set_output_low(&xslp_req);
    hw_set_output(&xslp_req);
    hw_set_input(&xslp_ind);
    
    /*** Initialize the message structure ***/
    msg.struc.time = 0;
    msg.struc.cnt = SEN_MSG_NUM_MEASUREMENTS;
    for(i=0; i<msg.struc.cnt; i++) {
        msg.struc.values[i].type = 0;
        msg.struc.values[i].value = 0;
    }
    
    /*** Initialize the hardware ***/
    /* Initialize the user LEDs and disable both by default */
    led_init();
    led1_high();
    led2_high();

    /* Initialize the ADC */
    adc_init(ADC_ADPS_16,ADC_REFS_VCC);

#if ENABLE_DBG
    /* Initialize UART1 for debug purposes */
    uart1_init();
    /* Initialize the printf function to use the uart1_putc() function for output */
    printf_init(uart1_putc);
#endif
    
    /* Initialize Xbee 3 (uses UART0) */
    xbee_init(9600UL);
    
    /* Give the hardware time to start up */
    led_show_startup();
    
    /* Print welcome message */
    printf("=== STARTING UP ... ===\n");
    
    /* Enable Watchdog (time: 8s) */
    wdt_enable(WDTO_8S);
    
    /*** Initialization of sensors and message value fields ***/
    /* ADC self check */
    msg.struc.values[MSG_VALUE_ADC_SELF].type = SEN_MSG_TYPE_CHK_ADC;
    msg.struc.values[MSG_VALUE_ADC_SELF].value = 0;
    /* MCU supply voltage */
    msg.struc.values[MSG_VALUE_MCU_VSS].type = SEN_MSG_TYPE_VSS_MCU;
    msg.struc.values[MSG_VALUE_MCU_VSS].value = 0;
    /* Battery voltage */
    msg.struc.values[MSG_VALUE_BAT_VSS].type = SEN_MSG_TYPE_VSS_BAT;
    msg.struc.values[MSG_VALUE_BAT_VSS].value = 0;
    /* Incident counter */
    msg.struc.values[MSG_VALUE_INCIDENT].type = SEN_MSG_TYPE_INCIDENTS;
    msg.struc.values[MSG_VALUE_INCIDENT].value = 0;
    /* Reboot sources */
    msg.struc.values[MSG_VALUE_REBOOOT].type = SEN_MSG_TYPE_REBOOT;
    msg.struc.values[MSG_VALUE_REBOOOT].value = MCUSR_dump & 0x0F;
    
    /* 103JT thermistor */
#if NODE_103JT_AVAILABLE
    msg.struc.values[MSG_VALUE_103JT_T].type = SEN_MSG_TYPE_TEMP_SURFACE;
    msg.struc.values[MSG_VALUE_103JT_T].value = 0;
#else
    msg.struc.values[MSG_VALUE_103JT_T].type = SEN_MSG_TYPE_IGNORE;
    msg.struc.values[MSG_VALUE_103JT_T].value = 0;
#endif

    /* Reset the WDT */
    wdt_reset();

    /* Initialize the TMP275 sensor */
    if(tmp275_init(&tmp275, TMP275_I2C_ADDRESS) != TMP275_RET_OK) {
        en_tmp275 = 0;
        msg.struc.values[MSG_VALUE_TMP275_T].type = SEN_MSG_TYPE_IGNORE;
        msg.struc.values[MSG_VALUE_TMP275_T].value = 0;
        printf("Couldn't initialize TMP275!\n");
    } else {
        en_tmp275 = 1;
        printf("... TMP275 ready\n");
    }
    
    /* Reset the WDT */
    wdt_reset();
    
#if NODE_CONFIGURATION==0
    /* Initialize the DS18B20 sensor */
    if(ds18x20_init(&ds18b20, &DDRD, &PORTD, &PIND, PD6) != DS18X20_RET_OK) {
        en_ds18b20 = 0;
        msg.struc.values[MSG_VALUE_DS18B20_T].type = SEN_MSG_TYPE_IGNORE;
        msg.struc.values[MSG_VALUE_DS18B20_T].value = 0;
        printf("Couldn't initialize DS18B20!\n");
    } else {
        en_ds18b20 = 1;
        printf("... DS18B20 ready\n");
    }
    
    /* Reset the WDT */
    wdt_reset();
    
    /* Initialize the STEMMA SOIL sensor */
    if(stemma_init(&stemma, STEMMA_I2C_ADDRESS) != STEMMA_RET_OK) {
        en_stemma = 0;
        msg.struc.values[MSG_VALUE_STEMMA_H].type = SEN_MSG_TYPE_IGNORE;
        msg.struc.values[MSG_VALUE_STEMMA_H].value = 0;
        printf("Couldn't initialize STEMMA!\n");
    } else {
        en_stemma = 1;
        printf("... STEMMA ready\n");
    }
#elif NODE_CONFIGURATION==1
    /* Initialize the AMS2302 sensor */
    dht_init(&am2302, &DDRD, &PORTD, &PIND, PD7, DHT_DEV_AM2302);
    en_am2302 = 1;
    printf("... AMS2302 ready\n");
#endif

    /* Reset the WDT */
    wdt_reset();

    /* Initialize the systick timer */
    systick_init();
    /* Set a systick callback function to be called every second */
    systick_set_callback_min(update);
    /* Enable interrupts globally */
    sei();

    /* Reset the WDT */
    wdt_reset();

    /* Reset the Xbee buffer */
    xbee_flush_rx();
    xbee_flush_tx();
    /* Check Xbee module connection */
    uint32_t time = 0;
    while(xbee_is_connected() != XBEE_RET_OK) {
        /* Check if timeout [s] has been reached (counter in [ms]) */
        if(time >= (XBEE_JOIN_TIMEOUT*1000)) {
            printf("Couldn't connect to the network ... aborting!\n");
            /* Run into endless loop and wait for watchdog reset */
            led1_low();
            led2_low();
            while(1);
        } else {
            /* Wait for some time */
            time += XBEE_JOIN_TIMEOUT_DELAY;
            _delay_ms(XBEE_JOIN_TIMEOUT_DELAY);
            /* Reset the WDT */
            wdt_reset();
        }
    }
    /* Print status message */
    printf("... ZIGBEE connected (message size = %d bytes)\n", SEN_MSG_SIZE);
    printf("\n");
    
    /* Main routine ... */
    while(1) {
        /*** Barrier synchronization */
        /* Wait until the barrier sync flag is set */
        while(barrier == 0) {
            _delay_ms(100);
            /* Reset the WDT */
            wdt_reset();
        }
        /* Reset barrier sync flag */
        barrier = 0;
        
        /* Wake-up xbee */
        hw_set_output_low(&xslp_req);
        time = 0;
        while(hw_read_input(&xslp_ind) == 0) {
            /* Check if timeout [s] has been reached (counter in [ms]) */
            if(time >= XBEE_WAKE_TIMEOUT) {
                printf("Couldn't wake-up xbee ... aborting!\n");
                /* Run into endless loop and wait for watchdog reset */
                led1_low();
                led2_low();
                while(1);
            } else {
                /* Wait for some time */
                time += XBEE_WAKE_TIMEOUT_DELAY;
                _delay_ms(XBEE_WAKE_TIMEOUT_DELAY);
                /* Reset the WDT */
                wdt_reset();
            }
        }
        printf("Local time: %03d days %02d:%02d:%02d.%03d\n", systick_get_day(), systick_get_hour(), systick_get_min(), systick_get_sec(), systick_get_msec());
        
        /*** ADC self-diagnosis (via ADC CH0) ***/
        /* Constant voltage divider (1:1) */
        msg.struc.values[MSG_VALUE_ADC_SELF].value = adc_read_input(ADC_CH0);
        printf("... ADC self-diagnosis: %d\n", msg.struc.values[MSG_VALUE_ADC_SELF].value);
        
        /*** MCU supply voltage (via ADC) ***/
        /* Supply voltage in volts (V) */
        vcc = adc_read_vcc();
        printf("... Supply voltage: %.2f\n", vcc);
        /* Pack measurement into msg as fixed-point number */
        msg.struc.values[MSG_VALUE_MCU_VSS].value = fp_float_to_fixed16_10to6(vcc);
        
        /*** Battery voltage (via ADC) ***/
        /* Supply voltage in volts (V) */
        adcres = adc_read_input(ADC_CH2);
        /* Calculate voltage from value (voltage divider 1:1) */
        measurement = 2.0 * (adcres * (vcc / 1023.0));
        printf("... Battery voltage: %.2f\n", measurement);
        /* Pack measurement into msg as fixed-point number */
        msg.struc.values[MSG_VALUE_BAT_VSS].value = fp_float_to_fixed16_10to6(measurement);

        /*** Xbee3 temperature ***/
        /* Reset the Xbee buffer */
        xbee_flush_rx();
        xbee_flush_tx();
        /* Temperature in degree Celsius (°C) */
        if(xbee_cmd_get_temperature(&measurement) == XBEE_RET_OK) {
            printf("... Xbee temperature: %.2f\n", measurement);
            /* Pack measurement into msg as fixed-point number */
            msg.struc.values[MSG_VALUE_XBEE_T].type = SEN_MSG_TYPE_TEMP_RADIO;
            msg.struc.values[MSG_VALUE_XBEE_T].value = fp_float_to_fixed16_10to6(measurement);
            /* Decrement incident counter */
            if(inc_xbee > 0) {
                inc_xbee--;
            }
        } else {
            msg.struc.values[MSG_VALUE_XBEE_T].type = SEN_MSG_TYPE_IGNORE;
            msg.struc.values[MSG_VALUE_XBEE_T].value = 0;
            printf("... Xbee temperature: FAILED!\n");
            /* Increment incident counter */
            if(inc_xbee < INCIDENT_SINGLE_MAX) {
                inc_xbee++;
            } else {
                /* Run into endless loop and wait for watchdog reset */
                led1_low();
                led2_low();
                while(1);
            }
        }

        /*** Xbee3 supply voltage ***/
        /* Reset the Xbee buffer */
        xbee_flush_rx();
        xbee_flush_tx();
        /* Supply voltage in volts (V) */
        if(xbee_cmd_get_vss(&measurement) == XBEE_RET_OK) {
            printf("... Xbee supply voltage: %.2f\n", measurement);
            /* Pack measurement into msg as fixed-point number */
            msg.struc.values[MSG_VALUE_XBEE_VSS].type = SEN_MSG_TYPE_VSS_RADIO;
            msg.struc.values[MSG_VALUE_XBEE_VSS].value = fp_float_to_fixed16_10to6(measurement);
            /* Decrement incident counter */
            if(inc_xbee > 0) {
                inc_xbee--;
            }
        } else {
            msg.struc.values[MSG_VALUE_XBEE_VSS].type = SEN_MSG_TYPE_IGNORE;
            msg.struc.values[MSG_VALUE_XBEE_VSS].value = 0;
            printf("... Xbee temperature: FAILED!\n");
            /* Increment incident counter */
            if(inc_xbee < INCIDENT_SINGLE_MAX) {
                inc_xbee++;
            } else {
                /* Run into endless loop and wait for watchdog reset */
                led1_low();
                led2_low();
                while(1);
            }
        }

#if NODE_103JT_AVAILABLE
        /*** 103JT thermistor (via ADC CH1) ***/
        /* Temperature in degree Celsius (°C) */
        measurement = jt103_get_temperature(adc_read_input(ADC_CH1));
        printf("... 103JT thermistor: %.2f\n", measurement);
        /* Pack measurement into msg as fixed-point number */
        msg.struc.values[MSG_VALUE_103JT_T].value = fp_float_to_fixed16_10to6(measurement);
#endif

        /* Reset the WDT */
        wdt_reset();

        /*** TMP275 ***/
        if(en_tmp275) {
            /* Temperature in degree Celsius (°C) */
            if(tmp275_get_temperature(&tmp275, &measurement) == TMP275_RET_OK) {
                printf("... TMP275 temperature: %.2f\n", measurement);
                /* Pack measurement into msg as fixed-point number */
                msg.struc.values[MSG_VALUE_TMP275_T].type = SEN_MSG_TYPE_TEMP_BOARD;
                msg.struc.values[MSG_VALUE_TMP275_T].value = fp_float_to_fixed16_10to6(measurement);
                /* Decrement incident counter */
                if(inc_tmp275 > 0) {
                    inc_tmp275--;
                }
            } else {
                msg.struc.values[MSG_VALUE_TMP275_T].type = SEN_MSG_TYPE_IGNORE;
                msg.struc.values[MSG_VALUE_TMP275_T].value = 0;
                printf("... TMP275 temperature: FAILED!\n");
                /* Increment incident counter */
                if(inc_tmp275 < INCIDENT_SINGLE_MAX) {
                    inc_tmp275++;
                } else {
                    en_tmp275 = 0;
                }
            }
        }

        /* Reset the WDT */
        wdt_reset();

#if NODE_CONFIGURATION==0
        /*** DS18B20 ***/
        if(en_ds18b20) {
            /* Temperature in degree Celsius (°C) */
            if(ds18x20_get_temperature(&ds18b20, &measurement) == DS18X20_RET_OK) {
                printf("... DS18B20 temperature: %.2f\n", measurement);
                /* Pack measurement into msg as fixed-point number */
                msg.struc.values[MSG_VALUE_DS18B20_T].type = SEN_MSG_TYPE_TEMP_SOIL;
                msg.struc.values[MSG_VALUE_DS18B20_T].value = fp_float_to_fixed16_10to6(measurement);
                /* Decrement incident counter */
                if(inc_ds18b20 > 0) {
                    inc_ds18b20--;
                }
            } else {
                msg.struc.values[MSG_VALUE_DS18B20_T].type = SEN_MSG_TYPE_IGNORE;
                msg.struc.values[MSG_VALUE_DS18B20_T].value = 0;
                printf("... DS18B20 temperature: FAILED!\n");
                /* Increment incident counter */
                if(inc_ds18b20 < INCIDENT_SINGLE_MAX) {
                    inc_ds18b20++;
                } else {
                    en_ds18b20 = 0;
                }
            }
        }

        /* Reset the WDT */
        wdt_reset();

        /*** STEMMA SOIL ***/
        if(en_stemma) {
            /* Relative humidity in percent (% RH) */
            if(stemma_get_humidity_avg(&stemma, &avg_stemma, &measurement) == STEMMA_RET_OK) {
                printf("... STEMMA humidity: %.2f\n", measurement);
                /* Pack measurement into msg as fixed-point number */
                msg.struc.values[MSG_VALUE_STEMMA_H].type = SEN_MSG_TYPE_HUMID_SOIL;
                msg.struc.values[MSG_VALUE_STEMMA_H].value = fp_float_to_fixed16_10to6(measurement);
                /* Decrement incident counter */
                if(inc_stemma > 0) {
                    inc_stemma--;
                }
            } else {
                msg.struc.values[MSG_VALUE_STEMMA_H].type = SEN_MSG_TYPE_IGNORE;
                msg.struc.values[MSG_VALUE_STEMMA_H].value = 0;
                printf("... STEMMA humidity: FAILED!\n");
                /* Increment incident counter */
                if(inc_stemma < INCIDENT_SINGLE_MAX) {
                    inc_stemma++;
                } else {
                    en_stemma = 0;
                }
            }
        }
#elif NODE_CONFIGURATION==1
        /*** AM2302 ***/
        if(en_am2302) {
            /* Temperature in degree Celsius (°C) */
            if(dht_get_temperature(&am2302, &measurement) == DHT_RET_OK) {
                printf("... AM2302 temperature: %.2f\n", measurement);
                /* Pack measurement into msg as fixed-point number */
                msg.struc.values[MSG_VALUE_AM2302_T].type = SEN_MSG_TYPE_TEMP_AIR;
                msg.struc.values[MSG_VALUE_AM2302_T].value = fp_float_to_fixed16_10to6(measurement);
                /* Decrement incident counter */
                if(inc_am2302 > 0) {
                    inc_am2302--;
                }
            } else {
                msg.struc.values[MSG_VALUE_AM2302_T].type = SEN_MSG_TYPE_IGNORE;
                msg.struc.values[MSG_VALUE_AM2302_T].value = 0;
                printf("... AM2302 temperature: FAILED!\n");
                /* Increment incident counter */
                if(inc_am2302 < INCIDENT_SINGLE_MAX) {
                    inc_am2302++;
                } else {
                    en_am2302 = 0;
                }
            }
            /* Relative humidity in percent (% RH) */
            if(dht_get_humidity(&am2302, &measurement) == DHT_RET_OK) {
                printf("... AM2302 humidity: %.2f\n", measurement);
                /* Pack measurement into msg as fixed-point number */
                msg.struc.values[MSG_VALUE_AM2302_H].type = SEN_MSG_TYPE_HUMID_AIR;
                msg.struc.values[MSG_VALUE_AM2302_H].value = fp_float_to_fixed16_10to6(measurement);
                /* Decrement incident counter */
                if(inc_am2302 > 0) {
                    inc_am2302--;
                }
            } else {
                msg.struc.values[MSG_VALUE_AM2302_H].type = SEN_MSG_TYPE_IGNORE;
                msg.struc.values[MSG_VALUE_AM2302_H].value = 0;
                printf("... AM2302 humidity: FAILED!\n");
                /* Increment incident counter */
                if(inc_am2302 < INCIDENT_SINGLE_MAX) {
                    inc_am2302++;
                } else {
                    en_am2302 = 0;
                }
            }
        }
#endif

        /* Reset the WDT */
        wdt_reset();

        /*** INCIDENT COUNTER ***/
        inc_sum = inc_xbee + inc_tmp275 + inc_ds18b20 + inc_stemma + inc_am2302;
        printf("... INCIDENT COUNTER (ENABLE):\n");
        printf("    ... XBEE:    %d\n", inc_xbee);
        printf("    ... TMP275:  %d (%d)\n", inc_tmp275, en_tmp275);
#if NODE_CONFIGURATION==0
        printf("    ... DS18B20: %d (%d)\n", inc_ds18b20, en_ds18b20);
        printf("    ... STEMMA:  %d (%d)\n", inc_stemma, en_stemma);
#elif NODE_CONFIGURATION==1
        printf("    ... AM2302:  %d (%d)\n", inc_am2302, en_am2302);
#endif
        printf("    ... TOTAL:   %d\n", inc_sum);
        /* Counter value between 0 and defined threshold */
        msg.struc.values[MSG_VALUE_INCIDENT].value = inc_sum;
        /* Check total incident counter */
        if(inc_sum >= INCIDENT_TOTAL_MAX) {
            /* Run into endless loop and wait for watchdog reset */
            led1_low();
            led2_low();
            while(1);
        }
        
        /* Reset the WDT */
        wdt_reset();
        
        /* Reset the Xbee buffer */
        xbee_flush_rx();
        xbee_flush_tx();
        /* Check Xbee module connection */
        time = 0;
        while(xbee_is_connected() != XBEE_RET_OK) {
            /* Check if timeout [s] has been reached (counter in [ms]) */
            if(time >= (XBEE_JOIN_TIMEOUT*1000)) {
                printf("Couldn't re-connect to the network ... aborting!\n");
                /* Run into endless loop and wait for watchdog reset */
                led1_low();
                led2_low();
                while(1);
            } else {
                /* Wait for some time */
                time += XBEE_JOIN_TIMEOUT_DELAY;
                _delay_ms(XBEE_JOIN_TIMEOUT_DELAY);
                /* Reset the WDT */
                wdt_reset();
            }
        }
        
        /* Send the measurement to the CH */
        int8_t ret = xbee_transmit_unicast(XBEE_DESTINATION_MAC, msg.byte, SEN_MSG_SIZE, 0x00);
        if(ret == XBEE_RET_OK) {
            printf("Sensor value update sent! (#%d)\n",msg.struc.time);
        } else {
            printf("ERROR sending message (%d)!\n",ret);
            /* Increment incident counter */
            if(inc_xbee < INCIDENT_SINGLE_MAX) {
                /* Severe issue, increment by 2 */
                inc_xbee += 2;
            } else {
                /* Run into endless loop and wait for watchdog reset */
                led1_low();
                led2_low();
                while(1);
            }
        }
        
        /* Send xbee to sleep */
        hw_set_output_high(&xslp_req);        
        
        /* Increment consecutive message counter */
        msg.struc.time++;
        printf("\n");
    }

    return 0;
}
