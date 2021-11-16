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
 * @version 1.1.2
 * @date    2021/10/25
 */


/*** APP CONFIGURATION ***/
#define ENABLE_DBG                  0               /**< Enable debug output via UART1 (9600 BAUD) */
#define UPDATE_INTERVAL             1               /**< Update interval [min] */
#define ASNX_VERSION_MINOR          4               /**< Minor version number of the used ASN(x) */
/* Enable (1) or disable (0) sensor measurements */
#define ENABLE_DS18B20              1               /**< enable DS18B20 sensor */
#define ENABLE_AM2302               0               /**< enable AM2302 sensor */
#define ENABLE_SHTC3                1               /**< enable SHTC3 sensor */
/* Check configuration */
#if (ENABLE_AM2302 && ENABLE_SHTC3)
#  error "Use either AM2302 or SHTC3 for air measurements, not both!"
#endif


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
#if ASNX_VERSION_MINOR>0
/* RTC */
#  include "rtc/pcf85263.h"
#else
/* SysTick */
#  include "timer/systick.h"
#endif
/* Sensors */
#include "sensors/tmp275.h"
#if ENABLE_DS18B20
#  include "sensors/ds18x20.h"
#endif
#if ENABLE_AM2302
#  include "sensors/dht.h"
#endif
#if ENABLE_SHTC3
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


/***** STRUCTURES *****************************************************/
/*!
 * A structure to store the sensor and fault indicator values.
 */
typedef struct {
    /* additional data */
    uint16_t time;          /**< timestamp (2 byte) */
    /* use case data */
    uint16_t t_air;         /**< air temperature (fixed point) */
    uint16_t t_soil;        /**< soil temperature (fixed point) */
    uint16_t h_air;         /**< air humidity (fixed point) */
    uint16_t h_soil;        /**< soil humidity (fixed point) */
    /* fault indicator */
    uint8_t x_nt;           /**< X_NT */
    uint8_t x_vs;           /**< X_VS */
    uint8_t x_bat;          /**< X_BAT */
    uint8_t x_art;          /**< X_ART */
    uint8_t x_rst;          /**< X_RST */
    uint8_t x_ic;           /**< X_IC */
    uint8_t x_adc;          /**< X_ADC */
    uint8_t x_usart;        /**< X_USART */
} MSG_t;


/***** GLOBAL VARIABLES ***********************************************/
#if ASNX_VERSION_MINOR==0
/*! Variable (flag) for barrier synchronization */
uint8_t barrier = 1;
#endif


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


#if ASNX_VERSION_MINOR==0
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
 * Debug: print the contents of a given sensor message structure
 */
#if ENABLE_DBG
void dbg_print_msg(MSG_t* msg) {
    printf("\n===== SENSOR MESSAGE CONTENTS =====\n");
    printf("%d message updates\n",msg->time);
    printf("=== SENSOR VALUES ===\n");
    printf("T_air:   %.2f C\n",fp_fixed16_to_float_10to6(msg->t_air));
    printf("T_soil:  %.2f C\n",fp_fixed16_to_float_10to6(msg->t_soil));
    printf("H_air:   %.2f %%\n",fp_fixed16_to_float_10to6(msg->h_air));
    printf("H_soil:  %.2f %%\n",fp_fixed16_to_float_10to6(msg->h_soil));
    printf("=== SENSOR VALUES ===\n");
    printf("X_NT:    %.2f\n",fp_fixed8_to_float_2to6(msg->x_nt));
    printf("X_VS:    %.2f\n",fp_fixed8_to_float_2to6(msg->x_vs));
    printf("X_BAT:   %.2f\n",fp_fixed8_to_float_2to6(msg->x_bat));
    printf("X_ART:   %.2f\n",fp_fixed8_to_float_2to6(msg->x_art));
    printf("X_RST:   %.2f\n",fp_fixed8_to_float_2to6(msg->x_rst));
    printf("X_IC:    %.2f\n",fp_fixed8_to_float_2to6(msg->x_ic));
    printf("X_ADC:   %.2f\n",fp_fixed8_to_float_2to6(msg->x_adc));
    printf("X_USART: %.2f\n",fp_fixed8_to_float_2to6(msg->x_usart));
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
    /* Soil moisture level is currently not used */
    msg.h_soil = 0;
#if ASNX_VERSION_MINOR>0
    /* Date/time structure */
    PCF85263_CNTTIME_t time = {0};
#endif
    /* Sensor handles */
    TMP275_t tmp275;                /* TMP275 sensor device structure */
#if ENABLE_DS18B20
    DS18X20_t ds18b20;              /* DS18B20 sensor device structure */
    uint8_t ds18b20_en = 0;         /* Sensor enable flag */
#endif
#if ENABLE_AM2302
    DHT_t am2302;                   /* AM2302 sensor device structure */
    uint8_t am2302_en = 0;          /* Sensor enable flag */
#endif
#if ENABLE_SHTC3
    SHTC3_t shtc3;                  /* SHTC3 sensor device structure */
    uint8_t shtc3_en = 0;           /* Sensor enable flag */
#endif
    /* Temporary sensor measurement variables */
    float measurement = 0.0;
#if (ENABLE_AM2302 || ENABLE_SHTC3)
    float measurement2 = 0.0;
#endif
    /* Diagnostic values */
    float v_bat, v_mcu, v_trx;
    float t_mcu, t_trx, t_brd;
    /* Runtime measurement */
    uint16_t runtime = 0, runtime_ms = 0;


    /*** 1.) initialize modules ***************************************/
    /* Print welcome message */
    printf("=== STARTING UP ... ===\n");
    
    /* Disable unused hardware modules */
#if ASNX_VERSION_MINOR>0
    PRR0 = _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRSPI);
    /* Configure the sleep mode */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
#else
    PRR0 = _BV(PRTIM2) | _BV(PRSPI);
#endif
    
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

#if ASNX_VERSION_MINOR>0
    /* Initialize the RTC */
    time.minutes = UPDATE_INTERVAL;
    if(pcf85263_init_wakeup_src(&time) != PCF85263_RET_OK) {
        printf("Couldn't initialize RTC ... aborting!\n");
        wait_for_wdt_reset();
    }
    /* Configure INT2 to fire interrupt when logic level is "low" */
    EICRA = 0x00;
    EIMSK = _BV(INT2);
#else
    /* Initialize the systick timer */
    systick_init();
    /* Set a systick callback function to be called every second */
    systick_set_callback_min(update);
#endif

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

#if ENABLE_DS18B20
    /* Initialize the DS18B20 sensor */
    if(ds18x20_init(&ds18b20, &DDRD, &PORTD, &PIND, PD6) != DS18X20_RET_OK) {
        printf("Couldn't initialize DS18B20!\n");
        ds18b20_en = 0;
    } else {
        ds18b20_en = 1;
    }
#endif

#if ENABLE_AM2302
    /* Initialize the AMS2302 sensor */
    dht_init(&am2302, &DDRD, &PORTD, &PIND, PD7, DHT_DEV_AM2302);
    printf("... AMS2302 ready\n");
    am2302_en = 1;
#endif

#if ENABLE_SHTC3
    /* Initialize the SHTC3 sensor */
    if(shtc3_init(&shtc3, SHTC3_I2C_ADDRESS) != SHTC3_RET_OK) {
        printf("Couldn't initialize SHTC3!\n");
        shtc3_en = 0;
    } else {
        shtc3_en = 1;
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
    printf("... ZIGBEE connected\n");


    while(1) {
#if ASNX_VERSION_MINOR>0
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
#else
        /*** 3.1) barrier synchronization *****************************/
        /* Wait until the barrier sync flag is set */
        while(barrier == 0) {
            _delay_ms(100);
        }
        /* Reset barrier sync flag */
        barrier = 0;
#endif

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
#if ENABLE_DS18B20
        /* Check if sensor is ready */
        if(ds18b20_en == 0) {
            /* Try to initialize sensor (again) */
            if(ds18x20_init(&ds18b20, &DDRD, &PORTD, &PIND, PD6) == DS18X20_RET_OK) {
                printf("Successfully (re-)initialized DS18B20!\n");
                ds18b20_en = 1;
            }
        }
        if(ds18b20_en == 1) {
            /* DS18B20 - Temperature in degree Celsius (°C) */
            if(ds18x20_get_temperature(&ds18b20, &measurement) == DS18X20_RET_OK) {
                printf("... DS18B20 temperature: %.2f\n", measurement);
                msg.t_soil = fp_float_to_fixed16_10to6(measurement);
                x_ic_dec(X_IC_DEC_NORM);
            } else {
                msg.t_soil = 0;
                x_ic_inc(X_IC_INC_NORM);
            }
        } else {
            msg.t_soil = 0;
        }
#endif

#if ENABLE_AM2302
        /* Check if sensor is ready */
        if(am2302_en == 0) {
            dht_init(&am2302, &DDRD, &PORTD, &PIND, PD7, DHT_DEV_AM2302);
            printf("Successfully (re-)initialized AM2302!\n");
            am2302_en = 1;
        }
        if(am2302_en == 1) {
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
        } else {
            msg.t_air = 0;
            msg.h_air = 0;
        }
#endif

#if ENABLE_SHTC3
        /* Check if sensor is ready */
        if(shtc3_en == 0) {
            /* Try to initialize sensor (again) */
            if(shtc3_init(&shtc3, SHTC3_I2C_ADDRESS) == SHTC3_RET_OK) {
                printf("Successfully (re-)initialized SHTC3!\n");
                shtc3_en = 1;
            }
        }
        if(shtc3_en == 1) {
            /* SHTC3 - Temperature in degree Celsius (°C) and relative humidity in percent (% RH) */
            if(shtc3_get_temperature_humidity(&shtc3, &measurement, &measurement2, 1) == SHTC3_RET_OK) {
                printf("... SHTC3 temperature: %.2f\n", measurement);
                printf("... SHTC3 humidity: %.2f\n", measurement2);
                msg.t_air = fp_float_to_fixed16_10to6(measurement);
                msg.h_air = fp_float_to_fixed16_10to6(measurement2);
                x_ic_dec(X_IC_DEC_NORM);
            } else {
                msg.t_air = 0;
                msg.h_air = 0;
                x_ic_inc(X_IC_INC_NORM);
            }
        } else {
            msg.t_air = 0;
            msg.h_air = 0;
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
        
        /* Node temperature monitor (X_NT) */
        msg.x_nt = fp_float_to_fixed8_2to6(x_nt_get_normalized(t_mcu, t_brd, t_trx));
        /* Supply voltage monitor (X_VS) */
        msg.x_vs = fp_float_to_fixed8_2to6(x_vs_get_normalized(v_mcu, v_trx));
        /* Battery voltage monitor (X_BAT) */
        msg.x_bat = fp_float_to_fixed8_2to6(x_bat_get_normalized(v_bat));
        /* Active runtime monitor (X_ART) */
        if(runtime > 0) {
            /* Subsequent cycle -> measurement available */
            // runtime_us = (uint32_t)(runtime) * 256UL;
            runtime_ms = (uint16_t)(((double)runtime * 256.0) / 1000.0);
            msg.x_art = fp_float_to_fixed8_2to6(x_art_get_normalized(runtime_ms));
        } else {
            msg.x_art = fp_float_to_fixed8_2to6(0.0);
        }
        /* Reset monitor (X_RST) */
        msg.x_rst = fp_float_to_fixed8_2to6(x_rst_get_normalized(MCUSR_dump & 0x0F));
        MCUSR_dump = 0;
        /* Software incident counter (X_IC) */
        msg.x_ic = fp_float_to_fixed8_2to6(x_ic_get_normalized());
        /* ADC self-check (X_ADC) */
        msg.x_adc = fp_float_to_fixed8_2to6(x_adc_get_normalized(diag_adc_check()));
        /* USART self-check (X_USART) */
        msg.x_usart = fp_float_to_fixed8_2to6(x_usart_get_normalized(NULL, NULL, 0));

        /* Check incident counter value */
        if(x_ic_get() >= X_IC_THRESHOLD) {
            printf("There were too many software incidents ... aborting!\n");
            /* Wait for watchdog reset */
            wait_for_wdt_reset();
        }


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
        /* Stop timer1 to save runtime measurement */
        timer1_stop();
        /* Save timer1 counter value */
        runtime = timer1_get_tcnt();
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


#if ASNX_VERSION_MINOR>0
        /*** 3.7) put MCU to sleep ************************************/
        sleep_enable();
        sleep_bod_disable();
        sleep_cpu();
#endif
    }

    return 0;
}


#if ASNX_VERSION_MINOR>0
/*!
 * INT2 external interrupt 2 interrupt.
 */
ISR(INT2_vect) {
    /* Actually not needed, but still ... */
    sleep_disable();
    /* Wait some time to fully wake up */
    _delay_ms(5);
}
#endif
