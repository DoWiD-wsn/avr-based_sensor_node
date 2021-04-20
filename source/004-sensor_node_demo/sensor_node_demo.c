/****
 * @brief   Demo application for a full sensor node.
 *
 * Demo application for an environmental monitoring sensor node that
 * periodically measures certain physical quantities and transmits the
 * data to a central cluster head via Zigbee (Xbee).
 *
 * @file    /004-sensor_node_demo/sensor_node_demo.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/20 $
 *****/


/*** DEMO CONFIGURATION ***/
/* Update interval [s] */
#define UPDATE_INTERVAL         (30)

/* MAC address of the destination */
#define XBEE_DESTINATION_MAC    (0x0013A20041B9FD07)

/* Incident counter total threshold */
#define INCIDENT_TOTAL_MAX      (100)
/* Incident counter single threshold */
#define INCIDENT_SINGLE_MAX     (10)

/* Sensor node configuration (0 ... stemma soil / 1 ... am2302) */
#define NODE_CONFIGURATION      (0)
/* Thermistor surface measurement available (0 ... no / 1 ... yes) */
#define NODE_103JT_AVAILABLE    (1)
/* Value to indicate incorrect measurment */
#define NODE_READING_FAILED     (-55.5)

/* Measurement message data indices */
#define MSG_VALUE_ADC_SELF      (0)
#define MSG_VALUE_MCU_VSS       (1)
#define MSG_VALUE_103JT_T       (2)
#define MSG_VALUE_TMP275_T      (3)
#define MSG_VALUE_DS18B20_T     (4)
#define MSG_VALUE_STEMMA_H      (5)
#define MSG_VALUE_AM2302_T      (6)
#define MSG_VALUE_AM2302_H      (7)
#define MSG_VALUE_INCIDENT      (8)
#define MSG_VALUE_REBOOOT       (9)


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
#include "util/printf.h"
#include "util/sensor_msg.h"


/***** GLOBAL VARIABLES ***********************************************/
/* Variable (flag) for barrier synchronization */
uint8_t barrier = 0;


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
    /* Temporary variable for sensor measurements */
    float measurement = 0.0;
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
    uint8_t en_ds18b20 = 0;         /**< DS18B20 is available (1) or has failed (0)  */
    uint8_t en_stemma = 0;          /**< STEMMA is available (1) or has failed (0) */
    uint8_t en_am2302 = 0;          /**< AM2302 is available (1) or has failed (0) */
    
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

    /* Initialize UART1 for debug purposes */
    uart1_init();
    /* Initialize the printf function to use the uart1_putc() function for output */
    printf_init(uart1_putc);
    
    /* Initialize Xbee 3 (uses UART0) */
    xbee_init(9600UL);
    
    /* Give the hardware time to start up */
    led_show_startup();
    
    /* Print welcome message */
    printf("=== STARTING UP ... ===\n");
    
    /* Enable Watchdog (time: 8s) */
    wdt_enable(WDTO_8S);
    
    /*** Get last reset source and store in msg structure ***/
    /* Get register value */
    msg.struc.values[MSG_VALUE_REBOOOT].value = MCUSR & 0x0F;
    /* Clear reset indicator bit(s) */
    MCUSR &= 0xF0;
    
    /*** Initialization of sensors and message value fields ***/
    /* ADC self check */
    msg.struc.values[MSG_VALUE_ADC_SELF].type = SEN_MSG_TYPE_CHK_ADC;
    msg.struc.values[MSG_VALUE_ADC_SELF].value = 0;
    /* MCU supply voltage */
    msg.struc.values[MSG_VALUE_MCU_VSS].type = SEN_MSG_TYPE_VSS_MCU;
    msg.struc.values[MSG_VALUE_MCU_VSS].value = 0;
    /* Incident counter */
    msg.struc.values[MSG_VALUE_INCIDENT].type = SEN_MSG_TYPE_INCIDENTS;
    msg.struc.values[MSG_VALUE_INCIDENT].value = 0;
    /* Reboot sources */
    msg.struc.values[MSG_VALUE_REBOOOT].type = SEN_MSG_TYPE_REBOOT;
    msg.struc.values[MSG_VALUE_REBOOOT].value = 0;
    
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
    systick_set_callback_sec(update);
    /* Enable interrupts globally */
    sei();

    /* Reset the WDT */
    wdt_reset();

    /* Check Xbee module connection */
    uint32_t time = 0;
    /* Check Xbee module connection */
    while(xbee_is_connected() != XBEE_RET_OK) {
        /* Check if timeout [s] has been reached (counter in [ms]) */
        if(time >= (XBEE_JOIN_TIMEOUT*1000)) {
            printf("Couldn't connect to the network ... aborting!\n");
            /* Run into endless loop and wait for watchdog reset */
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
    printf("... ZIGBEE connected\n");
    
    /* Main routine ... */
    while (1) {
        /*** Barrier synchronization */
        /* Wait until the barrier sync flag is set */
        while(barrier == 0) {
            _delay_ms(100);
            /* Reset the WDT */
            wdt_reset();
        }
        /* Reset barrier sync flag */
        barrier = 0;
        
        /*** ADC self-diagnosis (via ADC CH0) ***/
        /* Constant voltage divider (1:1) */
        msg.struc.values[MSG_VALUE_ADC_SELF].value = adc_read_input(ADC_CH0);
        printf("... ADC self-diagnosis: %d\n", msg.struc.values[MSG_VALUE_ADC_SELF].value);
        
        /*** MCU supply voltage (via ADC) ***/
        /* Supply voltage in volts (V) */
        measurement = adc_read_vcc();
        printf("... Supply voltage: %.2f\n", measurement);
        /* Pack measurement into msg as fixed-point number */
        msg.struc.values[MSG_VALUE_MCU_VSS].value = fp_float_to_fixed16_10to6(measurement);
        
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
            if(stemma_get_humidity(&stemma, &measurement) == STEMMA_RET_OK) {
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
        printf("    ... DS18B20: %d (%d)\n", inc_ds18b20, en_ds18b20);
        printf("    ... STEMMA:  %d (%d)\n", inc_stemma, en_stemma);
        printf("    ... AM2302:  %d (%d)\n", inc_am2302, en_am2302);
        printf("    ... TOTAL:   %d\n", inc_sum);
        /* Counter value between 0 and defined threshold */
        msg.struc.values[MSG_VALUE_INCIDENT].value = inc_sum;
        /* Check total incident counter */
        if(inc_sum >= INCIDENT_TOTAL_MAX) {
            /* Run into endless loop and wait for watchdog reset */
            while(1);
        }
        
        /* Reset the WDT */
        wdt_reset();
        
        /* Send the measurement to the CH */
        int8_t ret = xbee_transmit_unicast(XBEE_DESTINATION_MAC, msg.byte, SEN_MSG_SIZE, 0x00);
        if(ret == XBEE_RET_OK) {
            printf("Sensor value update sent!\n");
            /* Check the transmit response */
            uint8_t status;
            ret = xbee_transmit_status(&status);
            if(ret == XBEE_RET_OK) {
                if(ret == XBEE_TRANSMIT_STAT_DEL_OK) {
                    printf("... positive response received!\n");
                    /* Decrement incident counter */
                    if(inc_xbee > 0) {
                        inc_xbee--;
                    }
                } else {
                    printf("... NEGATIVE response received (%d)!\n",status);
                    /* Increment incident counter */
                    if(inc_xbee < INCIDENT_SINGLE_MAX) {
                        inc_xbee++;
                    } else {
                        /* Run into endless loop and wait for watchdog reset */
                        while(1);
                    }
                }
            } else {
                printf("ERROR receiving response (%d)!\n",ret);
                /* Increment incident counter */
                if(inc_xbee < INCIDENT_SINGLE_MAX) {
                    inc_xbee++;
                } else {
                    /* Run into endless loop and wait for watchdog reset */
                    while(1);
                }
            }
        } else {
            printf("ERROR sending message (%d)!\n",ret);
            /* Increment incident counter */
            if(inc_xbee < INCIDENT_SINGLE_MAX) {
                /* Severe issue, increment by 2 */
                inc_xbee+=2;
            } else {
                /* Run into endless loop and wait for watchdog reset */
                while(1);
            }
        }
        printf("\n");
    }

    return(0);
}
