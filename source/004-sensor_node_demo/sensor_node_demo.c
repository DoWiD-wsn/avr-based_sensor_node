/****
 * @brief   Demo application for a full sensor node incl. AVR/xbee sleep.
 *
 * Demo application for an environmental monitoring sensor node that
 * periodically measures certain physical quantities and transmits the
 * data to a central cluster head via Zigbee (Xbee). After the sensor
 * value update the Xbee and the AVR are put to sleep and are woken
 * up after a defined period by the onboard RTC. In case of an error
 * that cannot be resolved, the WDT is started and the MCU waits for a
 * WDT reset.
 *
 * @file    /004-sensor_node_demo/sensor_node_demo.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1.2 $
 * @date    $Date: 2021/05/19 $
 *
 * @todo    Add RTC timestamp functionality
 *****/


/*** DEMO CONFIGURATION ***/
/* Enable debug output via UART1 (9600 BAUD) */
#define ENABLE_DBG                  (1)

/* Update interval [min] */
#define UPDATE_INTERVAL             (10)

/* Enable (1) or disable (0) measurements/sensors */
#define ENABLE_ADC_SELF             (1)     /**< Enable the ADC self-test (via ADC) */
#define ENABLE_MCU_V                (1)     /**< Enable the MCU supply voltage (V) measurement (via ADC) */
#define ENABLE_BAT_V                (1)     /**< Enable the battery voltage (V) measurement (via ADC) */
#define ENABLE_XBEE_T               (1)     /**< Enable the XBEE radio temperature (T) measurement (via UART) */
#define ENABLE_XBEE_V               (1)     /**< Enable the XBEE radio supply voltage (V) measurement (via UART) */
#define ENABLE_103JT_T              (1)     /**< Enable the 103JT thermistor temperature (T) measurement (via ADC) */
#define ENABLE_TMP275_T             (1)     /**< Enable the TMP275 sensor temperature (T) measurement (via TWI) */
#define ENABLE_DS18B20_T            (1)     /**< Enable the DS18B20 sensor temperature (T) measurement (via OWI) */
#define ENABLE_STEMMA_H             (1)     /**< Enable the STEMMA SOIL sensor humidity (H) measurement (via TWI) */
#define ENABLE_AM2302_T             (0)     /**< Enable the AM2302 sensor temperature (T) measurement (via OWI) */
#define ENABLE_AM2302_H             (0)     /**< Enable the AM2302 sensor humidity (H) measurement (via OWI) */
#define ENABLE_INCIDENT             (1)     /**< Enable the transmission of the cumulative incident counter */
#define ENABLE_REBOOT               (1)     /**< Enable the transmission of the last reset source (MCUSR) */

/* MAC address of the destination */
#define XBEE_DESTINATION_MAC        SEN_MSG_MAC_CH

/* Incident counter total threshold */
#define INCIDENT_TOTAL_MAX          (100)
/* Incident counter single threshold */
#define INCIDENT_SINGLE_MAX         (10)

/* Measurement message data indices (derived from enable value above) */
#define MSG_VALUE_ADC_SELF          (0)
#define MSG_VALUE_MCU_V             (MSG_VALUE_ADC_SELF  + ENABLE_MCU_V)
#define MSG_VALUE_BAT_V             (MSG_VALUE_MCU_V     + ENABLE_BAT_V)
#define MSG_VALUE_XBEE_T            (MSG_VALUE_BAT_V     + ENABLE_XBEE_T)
#define MSG_VALUE_XBEE_V            (MSG_VALUE_XBEE_T    + ENABLE_XBEE_V)
#define MSG_VALUE_103JT_T           (MSG_VALUE_XBEE_V    + ENABLE_103JT_T)
#define MSG_VALUE_TMP275_T          (MSG_VALUE_103JT_T   + ENABLE_TMP275_T)
#define MSG_VALUE_DS18B20_T         (MSG_VALUE_TMP275_T  + ENABLE_DS18B20_T)
#define MSG_VALUE_STEMMA_H          (MSG_VALUE_DS18B20_T + ENABLE_STEMMA_H)
#define MSG_VALUE_AM2302_T          (MSG_VALUE_STEMMA_H  + ENABLE_AM2302_T)
#define MSG_VALUE_AM2302_H          (MSG_VALUE_AM2302_T  + ENABLE_AM2302_H)
#define MSG_VALUE_INCIDENT          (MSG_VALUE_AM2302_H  + ENABLE_INCIDENT)
#define MSG_VALUE_REBOOT            (MSG_VALUE_INCIDENT  + ENABLE_REBOOT)

/* Derive number of sensor values to be transmitted (passed to sensor_msg.h) */
#define SEN_MSG_NUM_MEASUREMENTS    (ENABLE_ADC_SELF + ENABLE_MCU_V + ENABLE_BAT_V + ENABLE_XBEE_T + ENABLE_XBEE_V + ENABLE_103JT_T + ENABLE_TMP275_T + ENABLE_DS18B20_T + ENABLE_STEMMA_H + ENABLE_AM2302_T + ENABLE_AM2302_H + ENABLE_INCIDENT + ENABLE_REBOOT)


/***** INCLUDES *******************************************************/
/*** AVR ***/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
/*** ASNX LIB ***/
/* MCU */
#if (ENABLE_ADC_SELF || ENABLE_MCU_V || ENABLE_BAT_V || ENABLE_103JT_T)
#include "adc/adc.h"
#endif
#include "hw/led.h"
#include "i2c/i2c.h"
#include "uart/uart.h"
/* Radio */
#include "xbee/xbee.h"
/* RTC */
#include "rtc/pcf85263.h"
/* Sensors */
// 103JT
#if ENABLE_103JT_T
#  include "sensors/jt103.h"
#endif
// TMP275
#if ENABLE_TMP275_T
#  include "sensors/tmp275.h"
TMP275_t tmp275;
#endif
// DS18B20
#if ENABLE_DS18B20_T
#  include "sensors/ds18x20.h"
DS18X20_t ds18b20;
#endif
// STEMMA
#if ENABLE_STEMMA_H
#  include "sensors/stemma_soil.h"
STEMMA_t stemma;
#endif
// AM2302
#if (ENABLE_AM2302_T || ENABLE_AM2302_H)
#  include "sensors/dht.h"
DHT_t am2302;
#endif
/* Misc */
#include "util/fixed_point.h"
#include "util/sensor_msg.h"
#if ENABLE_DBG
#  include "util/printf.h"
#else
#  define printf(...) do { } while (0)
#endif


/***** LOCAL FUNCTIONS ************************************************/
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
 * Activate WDT with minimal delay and wait for reset.
 ***/
void wait_for_wdt_reset(void) {
    /* Enable Watchdog (time: 15ms) */
    wdt_enable(WDTO_15MS);
    /* Wait for reset */
    while(1);
}


/***** MAIN ***********************************************************/
int main(void) {
    /*** Local variables ***/
    /* Message data structure */
    SEN_MSG_u msg;
    /* Date/time structure */
    PCF85263_CNTTIME_t time = {0};
    /* Temporary variable for sensor measurements */
    float measurement = 0.0, vcc = 0.0;
    /* Temporary variables */
    uint8_t reg = 0;
    uint8_t i = 0;
    /* Incident counter(s) and enable */
    uint16_t inc_xbee = 0;          /**< Incident counter for Xbee functions */
#if ENABLE_TMP275_T
    uint16_t inc_tmp275 = 0;        /**< Incident counter for TMP275 functions */
    uint8_t en_tmp275 = 0;          /**< TMP275 is available (1) or has failed (0) */
#endif
#if ENABLE_DS18B20_T
    uint16_t inc_ds18b20 = 0;       /**< Incident counter for DS18B20 functions */
    uint8_t en_ds18b20 = 0;         /**< DS18B20 is available (1) or has failed (0)  */
#endif
#if ENABLE_STEMMA_H
    uint16_t inc_stemma = 0;        /**< Incident counter for STEMMA SOIL functions */
    uint8_t en_stemma = 0;          /**< STEMMA is available (1) or has failed (0) */
    STEMMA_AVG_t avg_stemma = {0};  /**< Structure for the STEMMA floating average values */
#endif
#if (ENABLE_AM2302_T || ENABLE_AM2302_H)
    uint16_t inc_am2302 = 0;        /**< Incident counter for AM2302 functions */
    uint8_t en_am2302 = 0;          /**< AM2302 is available (1) or has failed (0) */
#endif
    uint16_t inc_sum = 0;           /**< Total incident counter (sum of others) */
    
    /* Disable unused hardware modules */
    PRR0 = _BV(PRTIM2) | _BV(PRTIM1) | _BV(PRTIM0) | _BV(PRSPI);
    PRR1 = _BV(PRTIM3);
    
    /* Configure the sleep mode */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    
    /*** Initialize the message structure ***/
    msg.struc.time = 0;
    msg.struc.cnt = SEN_MSG_NUM_MEASUREMENTS;
    for(i=0; i<SEN_MSG_NUM_MEASUREMENTS; i++) {
        msg.struc.values[i].type = 0;
        msg.struc.values[i].value = 0;
    }
    
    /*** Initialize the hardware ***/
    /* Initialize the user LEDs and disable both by default */
    led_init();
    led1_high();
    led2_high();

#if (ENABLE_ADC_SELF || ENABLE_MCU_V || ENABLE_BAT_V || ENABLE_103JT_T)
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
    uart1_init();
    /* Initialize the printf function to use the uart1_putc() function for output */
    printf_init(uart1_putc);
#else
    /* Disable UART1 */
    PRR0 |= _BV(PRUSART1);
#endif
    
    /* Initialize Xbee 3 (uses UART0) */
    xbee_init(9600UL);
    
    /* Print welcome message */
    printf("=== STARTING UP ... ===\n");
    
    /*** Setup the RTC as wake-up source for MCU ***/
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
    /* Reset time structure for stop-watch reset below */
    time.minutes = 0;
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
    
    /*** Initialization of sensors and message value fields ***/
#if ENABLE_ADC_SELF
    /* ADC self check */
    msg.struc.values[MSG_VALUE_ADC_SELF].type = SEN_MSG_TYPE_CHK_ADC;
    msg.struc.values[MSG_VALUE_ADC_SELF].value = 0;
#endif
#if ENABLE_MCU_V
    /* MCU supply voltage */
    msg.struc.values[MSG_VALUE_MCU_V].type = SEN_MSG_TYPE_VSS_MCU;
    msg.struc.values[MSG_VALUE_MCU_V].value = 0;
#endif
#if ENABLE_BAT_V
    /* Battery voltage */
    msg.struc.values[MSG_VALUE_BAT_V].type = SEN_MSG_TYPE_VSS_BAT;
    msg.struc.values[MSG_VALUE_BAT_V].value = 0;
#endif
#if ENABLE_103JT_T
    /* 103JT thermistor */
    msg.struc.values[MSG_VALUE_103JT_T].type = SEN_MSG_TYPE_TEMP_SURFACE;
    msg.struc.values[MSG_VALUE_103JT_T].value = 0;
#endif
#if ENABLE_INCIDENT
    /* Incident counter */
    msg.struc.values[MSG_VALUE_INCIDENT].type = SEN_MSG_TYPE_INCIDENTS;
    msg.struc.values[MSG_VALUE_INCIDENT].value = 0;
#endif
#if ENABLE_REBOOT
    /* Reboot sources */
    msg.struc.values[MSG_VALUE_REBOOT].type = SEN_MSG_TYPE_REBOOT;
    msg.struc.values[MSG_VALUE_REBOOT].value = MCUSR_dump & 0x0F;
#endif

#if ENABLE_TMP275_T
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
#endif
    
#if ENABLE_DS18B20_T
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
#endif
    
#if ENABLE_STEMMA_H
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
#endif
    
#if (ENABLE_AM2302_T || ENABLE_AM2302_H)
    /* Initialize the AMS2302 sensor */
    dht_init(&am2302, &DDRD, &PORTD, &PIND, PD7, DHT_DEV_AM2302);
    en_am2302 = 1;
    printf("... AMS2302 ready\n");
#endif

    /* Reset the Xbee buffer */
    xbee_rx_flush();
    xbee_tx_flush();
    /* Check Xbee module connection */
    uint32_t retries = 0;
    /* Check Xbee module connection */
    while(xbee_is_connected() != XBEE_RET_OK) {
        /* Check if timeout [s] has been reached (counter in [ms]) */
        if(retries >= (XBEE_JOIN_TIMEOUT*1000)) {
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
    printf("... ZIGBEE connected (message size = %d bytes)\n", SEN_MSG_SIZE);
    printf("\n");
    
    /* Main routine ... */
    while(1) {
        /* Wake-up xbee */
        if(xbee_sleep_disable() != XBEE_RET_OK) {
            printf("Couldn't wake-up xbee radio ... aborting!\n");
            /* Wait for watchdog reset */
            wait_for_wdt_reset();
        }
        
        /* Reset the TWI */
        i2c_reset();
#if (ENABLE_ADC_SELF || ENABLE_MCU_V || ENABLE_BAT_V || ENABLE_103JT_T)
        /* Enable ADC */
        adc_enable();
#endif
        /* Reset stop-watch time */
        pcf85263_set_stw_time(&time);
        
#if ENABLE_ADC_SELF
        /*** ADC self-diagnosis (via ADC CH0) ***/
        /* Constant voltage divider (1:1) */
        msg.struc.values[MSG_VALUE_ADC_SELF].value = adc_read_input(ADC_CH0);
        printf("... ADC self-diagnosis: %d\n", msg.struc.values[MSG_VALUE_ADC_SELF].value);
#endif
        
#if ENABLE_MCU_V
        /*** MCU supply voltage (via ADC) ***/
        /* Supply voltage in volts (V) */
        vcc = adc_read_vcc();
        printf("... Supply voltage: %.2f\n", vcc);
        /* Pack measurement into msg as fixed-point number */
        msg.struc.values[MSG_VALUE_MCU_V].value = fp_float_to_fixed16_10to6(vcc);
#endif
        
#if ENABLE_BAT_V
        /*** Battery voltage (via ADC) ***/
        /* Calculate voltage from value (voltage divider 1:1) */
        measurement = 2.0 * (adc_read_input(ADC_CH2) * (vcc / 1023.0));
        printf("... Battery voltage: %.2f\n", measurement);
        /* Pack measurement into msg as fixed-point number */
        msg.struc.values[MSG_VALUE_BAT_V].value = fp_float_to_fixed16_10to6(measurement);
#endif
        
#if ENABLE_XBEE_T
        /*** Xbee3 temperature ***/
        /* Reset the Xbee buffer */
        xbee_rx_flush();
        xbee_tx_flush();
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
                /* Wait for watchdog reset */
                wait_for_wdt_reset();
            }
        }
#endif
        
#if ENABLE_XBEE_V
        /*** Xbee3 supply voltage ***/
        /* Reset the Xbee buffer */
        xbee_rx_flush();
        xbee_tx_flush();
        /* Supply voltage in volts (V) */
        if(xbee_cmd_get_vss(&measurement) == XBEE_RET_OK) {
            printf("... Xbee supply voltage: %.2f\n", measurement);
            /* Pack measurement into msg as fixed-point number */
            msg.struc.values[MSG_VALUE_XBEE_V].type = SEN_MSG_TYPE_VSS_RADIO;
            msg.struc.values[MSG_VALUE_XBEE_V].value = fp_float_to_fixed16_10to6(measurement);
            /* Decrement incident counter */
            if(inc_xbee > 0) {
                inc_xbee--;
            }
        } else {
            msg.struc.values[MSG_VALUE_XBEE_V].type = SEN_MSG_TYPE_IGNORE;
            msg.struc.values[MSG_VALUE_XBEE_V].value = 0;
            printf("... Xbee temperature: FAILED!\n");
            /* Increment incident counter */
            if(inc_xbee < INCIDENT_SINGLE_MAX) {
                inc_xbee++;
            } else {
                /* Wait for watchdog reset */
                wait_for_wdt_reset();
            }
        }
#endif
        
#if ENABLE_103JT_T
        /*** 103JT thermistor (via ADC CH1) ***/
        /* Temperature in degree Celsius (°C) */
        measurement = jt103_get_temperature(adc_read_input(ADC_CH1));
        printf("... 103JT thermistor: %.2f\n", measurement);
        /* Pack measurement into msg as fixed-point number */
        msg.struc.values[MSG_VALUE_103JT_T].value = fp_float_to_fixed16_10to6(measurement);
#endif

#if ENABLE_TMP275_T
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
#endif

#if ENABLE_DS18B20_T
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
#endif
        
#if ENABLE_STEMMA_H
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
#endif
        
#if ENABLE_AM2302_T
        /*** AM2302 (T) ***/
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
        }
#endif
        
#if ENABLE_AM2302_H
        /*** AM2302 (H) ***/
        if(en_am2302) {
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

        /*** INCIDENT COUNTER ***/
        inc_sum = inc_xbee;
#if ENABLE_TMP275_T
        inc_sum += inc_tmp275;
#endif
#if ENABLE_DS18B20_T
        inc_sum += inc_ds18b20;
#endif
#if ENABLE_STEMMA_H
        inc_sum += inc_stemma;
#endif
#if (ENABLE_AM2302_T || ENABLE_AM2302_H)
        inc_sum += inc_am2302;
#endif

        printf("... INCIDENT COUNTER (ENABLE):\n");
        printf("    ... XBEE:    %d\n", inc_xbee);
#if ENABLE_TMP275_T
        printf("    ... TMP275:  %d (%d)\n", inc_tmp275, en_tmp275);
#endif
#if ENABLE_DS18B20_T
        printf("    ... DS18B20: %d (%d)\n", inc_ds18b20, en_ds18b20);
#endif
#if ENABLE_STEMMA_H
        printf("    ... STEMMA:  %d (%d)\n", inc_stemma, en_stemma);
#endif
#if (ENABLE_AM2302_T || ENABLE_AM2302_H)
        printf("    ... AM2302:  %d (%d)\n", inc_am2302, en_am2302);
#endif
        printf("    ... TOTAL:   %d\n", inc_sum);
        /* Counter value between 0 and defined threshold */
        msg.struc.values[MSG_VALUE_INCIDENT].value = inc_sum;
        /* Check total incident counter */
        if(inc_sum >= INCIDENT_TOTAL_MAX) {
            /* Wait for watchdog reset */
            wait_for_wdt_reset();
        }
        
        /* Reset the Xbee buffer */
        xbee_rx_flush();
        xbee_tx_flush();
        /* Check Xbee module connection */
        retries = 0;
        while(xbee_is_connected() != XBEE_RET_OK) {
            /* Check if timeout [s] has been reached (counter in [ms]) */
            if(retries >= (XBEE_JOIN_TIMEOUT*1000)) {
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
        int8_t ret = xbee_transmit_unicast(XBEE_DESTINATION_MAC, msg.byte, SEN_MSG_SIZE, 0x00);
        if(ret != XBEE_RET_OK) {
            printf("ERROR sending message (%d)!\n",ret);
            /* Increment incident counter */
            if(inc_xbee < INCIDENT_SINGLE_MAX) {
                /* Severe issue, increment by 2 */
                inc_xbee += 2;
            } else {
                /* Wait for watchdog reset */
                wait_for_wdt_reset();
            }
        }
        /* Wait until the xbee/uart0 TX buffer is empty */
        retries = 0;
        while(xbee_tx_cnt()>0) {
            /* Check if timeout has been reached */
            if(retries >= 100) {
                printf("UART0 TX buffer didn't become empty ... aborting!\n");
                /* Wait for watchdog reset */
                wait_for_wdt_reset();
            } else {
                /* Wait for some time */
                retries++;
                _delay_ms(1);
            }
        }
        printf("Sensor value update sent! (#%d)\n\n",msg.struc.time++);
        
        /* Send xbee to sleep */
        if(xbee_sleep_enable() != XBEE_RET_OK) {
            printf("Couldn't send xbee radio to sleep ... aborting!\n");
            /* Wait for watchdog reset */
            wait_for_wdt_reset();
        }
        
#if (ENABLE_ADC_SELF || ENABLE_MCU_V || ENABLE_BAT_V || ENABLE_103JT_T)
        /* Disable ADC */
        adc_disable();
#endif
        
        /* Enter power down mode */
        sleep_enable();
        sleep_bod_disable();
        sleep_cpu();
    }

    return 0;
}


/***** ISR ************************************************************/
ISR(INT2_vect) {
    /* Actually not needed, but still ... */
    sleep_disable();
    /* Wait some time to fully wake up */
    _delay_ms(5);
}
