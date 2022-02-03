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
 * @version 2.1.3
 * @date    2022/02/01
 */


/*** APP CONFIGURATION ***/
#define ENABLE_DBG                  0               /**< Enable debug output via UART1 (9600 BAUD) */
#define ENABLE_DBG_MSG              0               /**< Enable debug output of message content */
#define ENABLE_DBG_INDICATOR        0               /**< Enable debug output of indicator values */
#define UPDATE_INTERVAL             10              /**< Update interval [min] */
#define ASNX_VERSION_MINOR          4               /**< Minor version number of the used ASN(x) */
/* Zigbee network */
#define ZIGBEE_ENABLE_ACK           0               /**< Enable (1) or disable (0) transmit response */
/* Enable (1) or disable (0) sensor measurements */
#define ENABLE_DS18B20              0               /**< enable DS18B20 sensor */
#define ENABLE_AM2302               1               /**< enable AM2302 sensor */
#define ENABLE_SHTC3                0               /**< enable SHTC3 sensor */
/* Check configuration */
#if (ENABLE_AM2302 && ENABLE_SHTC3)
#  error "Use either AM2302 or SHTC3 for air measurements, not both!"
#endif


/***** INCLUDES *******************************************************/
/*** STD ***/
#include <math.h>
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
 * A structure to store the sensor and indicator values.
 */
typedef struct {
    /* additional data */
    uint16_t time;          /**< timestamp (2 byte) */
    /* use case data */
    uint16_t t_air;         /**< air temperature (fixed point) */
    uint16_t h_air;         /**< air humidity (fixed point) */
    uint16_t t_soil;        /**< soil temperature (fixed point) */
    uint16_t h_soil;        /**< soil humidity (fixed point) */
    /* battery SoC */
    uint8_t soc;            /**< battery state-of-charge (fixed point) */
    /* indicator */
    uint8_t danger;         /**< danger indicator (fixed point) */
    uint8_t safe;           /**< safe indicator (fixed point) */
} MSG_t;


/***** GLOBAL VARIABLES ***********************************************/
#if ASNX_VERSION_MINOR==0
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
#if ASNX_VERSION_MINOR==0
void update(void);
#endif
#if ENABLE_DBG_MSG
void dbg_print_msg(MSG_t* msg);
#endif
#if ENABLE_DBG_INDICATOR
void dbg_print_indicator(float danger, float safe, float x_nt, float x_vs, float x_bat, float x_art, float x_rst, float x_ic, float x_adc, float x_usart, float t_air_i, float h_air_i, float t_soil_i, float h_soil_i);
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
#if ENABLE_DBG_MSG
void dbg_print_msg(MSG_t* msg) {
    printf("\n===== SENSOR MESSAGE CONTENTS =====\n");
    printf("%d message updates\n",msg->time);
    printf("=== SENSOR VALUES ===\n");
    printf("T_air: %.2f C\n",fp_fixed16_to_float_10to6(msg->t_air));
    printf("H_air: %.2f %%\n",fp_fixed16_to_float_10to6(msg->h_air));
    printf("T_soil: %.2f C\n",fp_fixed16_to_float_10to6(msg->t_soil));
    printf("H_soil: %.2f %%\n",fp_fixed16_to_float_10to6(msg->h_soil));
    printf("=== BATTERY ===\n");
    printf("SoC: %d\n",msg->soc);
    printf("=== SENSOR VALUES ===\n");
    printf("Danger: %.2f\n",fp_fixed8_to_float_2to6(msg->danger));
    printf("Safe: %.2f\n",fp_fixed8_to_float_2to6(msg->safe));
    printf("===================================\n\n");
}
#endif


/*!
 * Debug: print the values of the single indicator shares
 */
#if ENABLE_DBG_INDICATOR
void dbg_print_indicator(float danger, float safe, float x_nt, float x_vs, float x_bat, float x_art, float x_rst, float x_ic, float x_adc, float x_usart, float t_air_i, float h_air_i, float t_soil_i, float h_soil_i) {
    printf("\n===== INDICATOR VALUES =====\n");
    printf("danger: %.2f\n",danger);
    printf("safe: %.2f\n",safe);
    printf("=== DANGER/FAULT INDICATORS ===\n");
    printf("X_NT: %.2f\n",x_nt);
    printf("X_VS: %.2f\n",x_vs);
    printf("X_BAT: %.2f\n",x_bat);
    printf("X_ART: %.2f\n",x_art);
    printf("X_RST: %.2f\n",x_rst);
    printf("X_IC: %.2f\n",x_ic);
    printf("X_ADC: %.2f\n",x_adc);
    printf("X_UART: %.2f\n",x_usart);
    printf("=== SAFE INDICATORS ===\n");
    printf("T_AIR: %.2f\n",t_air_i);
    printf("H_AIR: %.2f\n",h_air_i);
    printf("T_SOIL: %.2f\n",t_soil_i);
    printf("H_SOIL: %.2f\n",h_soil_i);
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
    MSG_t msg = {0};
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
#if (ENABLE_DS18B20 || ENABLE_AM2302 || ENABLE_SHTC3)
    float measurement = 0.0;
#endif
#if (ENABLE_AM2302 || ENABLE_SHTC3)
    float measurement2 = 0.0;
#endif
    /* Diagnostic values */
    float v_bat=0.0, v_mcu=0.0, v_trx=0.0;
    float t_mcu=0.0, t_trx=0.0, t_brd=0.0;
    /* Sensor values */
    float t_air_t=0.0, h_air_t=0.0, t_soil_t=0.0, h_soil_t=0.0;
    /* Fault indicator */
    float x_nt=0.0, x_vs=0.0, x_bat=0.0, x_art=0.0, x_rst=0.0, x_ic=0.0, x_adc=0.0, x_usart=0.0;
    /* Safe indicator */
    float t_air_i=0.0, h_air_i=0.0, t_soil_i=0.0, h_soil_i=0.0;
    safe_t t_air_s, h_air_s, t_soil_s, h_soil_s;
    /* Aggregated indicator */
    float danger=0.0, safe=0.0;
    /* Runtime measurement */
    uint16_t runtime = 0, runtime_ms = 0;
    /* Function return value */
    int8_t ret = 0;


    /*** 1.) initialize modules ***************************************/
    /* Disable WDT just to be sure */
    wdt_disable();
    /* Disable unused hardware modules */
#if ASNX_VERSION_MINOR>0
    PRR0 = _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRSPI);
    /* Configure the sleep mode */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
#else
    PRR0 = _BV(PRTIM2) | _BV(PRSPI);
#endif
    
#if ENABLE_DBG
    /* Initialize UART1 for debug purposes (9600 baud) */
    uart1_init(9600UL);
    /* Initialize the printf function to use the uart1_write_char() function for output */
    printf_init(uart1_write_char);
#else
    /* Disable UART1 */
    PRR0 |= _BV(PRUSART1);
#endif
    /* Print welcome message */
    printf("=== STARTING UP ... ===\n");
    
    /* Initialize the user LEDs and disable both by default */
    led_init();
    /* Initialize the ADC */
    adc_init(ADC_ADPS_32,ADC_REFS_VCC);
    adc_disable_din(0x07);
    /* Initialize I2C master interface */
    i2c_init();
    /* Initialize Xbee 3 (uses UART0 @9600 baud; receive via ISR) */
    uart0_init(9600UL);
    uart0_interrupt_enable();
    xbee_init(uart0_write_byte, uart0_pop_byte, uart0_rx_buffer_cnt);
    /* Status message */
    printf("Communication interfaces initialized ...\n");
    
    /* Initialize the diagnostic circuitry */
    diag_init();
    diag_disable();
    /* Initialize the fault indicators */
    indicators_init();
    /* Initialize the safe indicators */
    safe_init(&t_air_s);
    safe_init(&h_air_s);
    safe_init(&t_soil_s);
    safe_init(&h_soil_s);
    /* Status message */
    printf("Indicators initialized ...\n");

#if ASNX_VERSION_MINOR>0
    /* Initialize the RTC */
    time.minutes = UPDATE_INTERVAL;
    ret = pcf85263_init_wakeup_src(&time);
    if(ret != PCF85263_RET_OK) {
        printf("Couldn't initialize RTC (%d) ... aborting!\n",ret);
        sleep_until_reset(WDTO_15MS);
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
    ret = tmp275_init(&tmp275, TMP275_I2C_ADDRESS);
    if(ret != TMP275_RET_OK) {
        printf("Couldn't initialize TMP275 (%d) ... aborting!\n",ret);
        sleep_until_reset(WDTO_15MS);
    }
    /* Configure the TMP275 sensor (SD; 10-bit mode) */
    ret = tmp275_set_config(&tmp275, 0x21);
    if(ret != TMP275_RET_OK) {
        printf("Couldn't configure TMP275 (%d) ... aborting!\n",ret);
        sleep_until_reset(WDTO_15MS);
    }
    /* Status message */
    printf("TMP275 initialized ...\n");

#if ENABLE_DS18B20
    /* Initialize the DS18B20 sensor */
    ret = ds18x20_init(&ds18b20, &DDRD, &PORTD, &PIND, PD6);
    if(ret != DS18X20_RET_OK) {
        printf("Couldn't initialize DS18B20 (%d)!\n",ret);
        ds18b20_en = 0;
    } else {
        printf("DS18B20 initialized ...\n");
        ds18b20_en = 1;
    }
#endif

#if ENABLE_AM2302
    /* Initialize the AMS2302 sensor */
    ret = dht_init(&am2302, &DDRD, &PORTD, &PIND, PD7, DHT_DEV_AM2302);
    if(ret != DHT_RET_OK) {
        printf("Couldn't initialize AMS2302 (%d)!\n",ret);
        am2302_en = 0;
    } else {
        printf("AMS2302 initialized ...\n");
        am2302_en = 1;
    }
#endif

#if ENABLE_SHTC3
    /* Initialize the SHTC3 sensor */
    ret = shtc3_init(&shtc3, SHTC3_I2C_ADDRESS);
    if(ret != SHTC3_RET_OK) {
        printf("Couldn't initialize SHTC3! (%d)\n",ret);
        shtc3_en = 0;
    } else {
        printf("SHTC3 initialized ...\n");
        shtc3_en = 1;
    }
#endif

    
    /*** 2.) connect to the Zigbee network ****************************/
    /* Check Xbee module connection */
    printf("Connecting to Zigbee network ... ");
    ret = xbee_wait_for_connected();
    if(ret != XBEE_RET_OK) {
        printf("\nCouldn't connect to the network (%d) ... aborting!\n",ret);
        /* Wait for watchdog reset */
        sleep_until_reset(WDTO_8S);
    }
    /* Print status message */
    printf("connected\n");


    while(1) {
        /*** (Re-)enable I2C interface ***/
        /* Reset the TWI */
        i2c_reset();
        
#if ASNX_VERSION_MINOR>0
        /*** 3.1) reset RTC (stop-watch mode) *************************/
        /* Stop RTC */
        ret = pcf85263_stop();
        if(ret != PCF85263_RET_OK) {
            printf("Couldn't stop RTC (%d) ... aborting!\n",ret);
            sleep_until_reset(WDTO_15MS);
        }
        /* Reset time structure for stop-watch reset below */
        pcf85263_clear_stw_time(&time);
        /* Reset stop-watch time */
        pcf85263_set_stw_time(&time);
        /* Start RTC */
        ret = pcf85263_start();
        if(ret != PCF85263_RET_OK) {
            printf("Couldn't re-start RTC (%d) ... aborting!\n",ret);
            sleep_until_reset(WDTO_15MS);
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
        ret = xbee_sleep_disable();
        if(ret != XBEE_RET_OK) {
            printf("Couldn't wake-up xbee radio (%d) ... aborting!\n",ret);
            /* Wait for watchdog reset */
            sleep_until_reset(WDTO_15MS);
        }
        
        /* Start timer1 with prescaler 1024 -> measurement interval [256us; 16.78s] */
        timer1_start(TIMER_PRESCALER_1024);
        
        /* Enable ADC */
        adc_enable();
        /* Enable the self-diagnostics */
        diag_enable();

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
            ret = ds18x20_get_temperature(&ds18b20, &measurement);
            if(ret == DS18X20_RET_OK) {
                printf("... DS18B20 temperature: %.2f\n", measurement);
                t_soil_t = measurement;
                msg.t_soil = fp_float_to_fixed16_10to6(t_soil_t);
                x_ic_dec(X_IC_DEC_NORM);
            } else {
                printf("... DS18B20 reading failed (%d)\n",ret);
                t_soil_t = 0.0;
                msg.t_soil = 0;
                x_ic_inc(X_IC_INC_NORM);
            }
        } else {
            t_soil_t = 0.0;
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
            ret = dht_get_temperature_humidity(&am2302, &measurement, &measurement2);
            if(ret == DHT_RET_OK) {
                printf("... AM2302 temperature: %.2f\n", measurement);
                printf("... AM2302 humidity: %.2f\n", measurement2);
                t_air_t = measurement;
                h_air_t = measurement2;
                msg.t_air = fp_float_to_fixed16_10to6(t_air_t);
                msg.h_air = fp_float_to_fixed16_10to6(h_air_t);
                x_ic_dec(X_IC_DEC_NORM);
            } else {
                printf("... AM2302 reading failed (%d)\n",ret);
                t_air_t = 0.0;
                h_air_t = 0.0;
                msg.t_air = 0;
                msg.h_air = 0;
                x_ic_inc(X_IC_INC_NORM);
            }
        } else {
            t_air_t = 0.0;
            h_air_t = 0.0;
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
            ret = shtc3_get_temperature_humidity(&shtc3, &measurement, &measurement2, 1);
            if(ret == SHTC3_RET_OK) {
                printf("... SHTC3 temperature: %.2f\n", measurement);
                printf("... SHTC3 humidity: %.2f\n", measurement2);
                t_air_t = measurement;
                h_air_t = measurement2;
                msg.t_air = fp_float_to_fixed16_10to6(t_air_t);
                msg.h_air = fp_float_to_fixed16_10to6(h_air_t);
                x_ic_dec(X_IC_DEC_NORM);
            } else {
                printf("... SHTC3 reading failed (%d)\n",ret);
                t_air_t = 0.0;
                h_air_t = 0.0;
                msg.t_air = 0;
                msg.h_air = 0;
                x_ic_inc(X_IC_INC_NORM);
            }
        } else {
            t_air_t = 0.0;
            h_air_t = 0.0;
            msg.t_air = 0;
            msg.h_air = 0;
        }
#endif


        /*** 3.4) perform self-diagnostics ****************************/
        /* Trigger TMP275 conversion (wait 55ms before reading) */
        uint8_t tmp275_ret = tmp275_set_config(&tmp275, 0xA1);
        /* MCU surface temperature (103JT thermistor via ADC CH2) */
        t_mcu = diag_tsurface_read();
        /* Xbee3 temperature */
        uart0_rx_cb_flush();
        ret = xbee_cmd_get_temperature(&t_trx);
        if(ret == XBEE_RET_OK) {
            x_ic_dec(X_IC_DEC_NORM);
        } else {
            printf("... XBee temperature reading failed (%d)\n",ret);
            x_ic_inc(X_IC_INC_NORM);
        }
        /* Board temperature (TMP275 via TWI) */
        if(tmp275_ret == TMP275_RET_OK) {
            /* Get temperature in degree Celsius (°C) */
            ret = tmp275_get_temperature(&tmp275, &t_brd);
            if(ret == TMP275_RET_OK) {
                x_ic_dec(X_IC_DEC_NORM);
            } else {
                printf("... Board temperature reading failed (%d)\n",ret);
                x_ic_inc(X_IC_INC_NORM);
            }
        }  else {
            printf("... TMP275 start measurement failed (%d)\n",tmp275_ret);
            x_ic_inc(X_IC_INC_NORM);
        }
        
        /* MCU supply voltage in volts (V) */
        v_mcu = diag_vcc_read();
        /* Xbee3 supply voltage */
        uart0_rx_cb_flush();
        ret = xbee_cmd_get_vss(&v_trx);
        if(ret == XBEE_RET_OK) {
            x_ic_dec(X_IC_DEC_NORM);
        } else {
            printf("... XBee voltage reading failed (%d)\n",ret);
            x_ic_inc(X_IC_INC_NORM);
        }
        /* Battery voltage (via ADC) */
        v_bat = diag_vbat_read(v_mcu);
        
        /*** fault indicator ***/
        /* Node temperature monitor (X_NT) */
        x_nt = x_nt_get_normalized(t_mcu, t_brd, t_trx);
        /* Supply voltage monitor (X_VS) */
        x_vs = x_vs_get_normalized(v_mcu, v_trx);
        /* Battery voltage monitor (X_BAT) */
        x_bat = x_bat_get_normalized(v_bat);
        /* Active runtime monitor (X_ART) */
        if(runtime > 0) {
            /* Subsequent cycle -> measurement available */
            runtime_ms = (uint16_t)((double)runtime * 0.256);
            x_art = x_art_get_normalized(runtime_ms);
        } else {
            x_art = 0.0;
        }
        /* Reset monitor (X_RST) */
        x_rst = x_rst_get_normalized(MCUSR_dump & 0x0F);
        MCUSR_dump = 0;
        /* Software incident counter (X_IC) */
        x_ic = x_ic_get_normalized();
        /* ADC self-check (X_ADC) */
        x_adc = x_adc_get_normalized(diag_adc_check());
        /* USART self-check (X_USART) */
        x_usart = x_usart_get_normalized(NULL, NULL, 0);
        
        /*** danger indicator ***/
        danger = fmin(1, (x_nt + x_vs + x_bat + x_art + x_rst + x_ic + x_adc + x_usart));
        msg.danger = fp_float_to_fixed8_2to6(danger);
        
        /*** safe indicator ***/
        /* Get the updated safe indicator values */
        t_air_i = safe_update(&t_air_s, t_air_t);
        h_air_i = safe_update(&h_air_s, h_air_t);
        t_soil_i = safe_update(&t_soil_s, t_soil_t);
        h_soil_i = safe_update(&h_soil_s, h_soil_t);
        /* Get the aggregated value */
        safe = exp(-fmax(fmax(t_air_i,h_air_i), fmax(t_soil_i,h_soil_i))*SAFE_SENS);
        msg.safe = fp_float_to_fixed8_2to6(safe);

#if ENABLE_DBG_INDICATOR
        /* Print the indicator values */
        dbg_print_indicator(danger, safe, x_nt, x_vs, x_bat, x_art, x_rst, x_ic, x_adc, x_usart, t_air_i, h_air_i, t_soil_i, h_soil_i);
#endif

        /* Battery SoC */
        msg.soc = diag_vbat_soc(x_bat_get_mean());

        /* Check incident counter value */
        if(x_ic_get() >= X_IC_THRESHOLD) {
            printf("There were too many software incidents ... aborting!\n");
            /* Wait for watchdog reset */
            sleep_until_reset(WDTO_15MS);
        }


        /*** 3.5) send values via Zigbee ******************************/
        /* Check Xbee module connection */
        printf("Check Zigbee network connection ... ");
        uart0_rx_cb_flush();
        ret = xbee_wait_for_reconnected();
        if(ret != XBEE_RET_OK) {
            printf("\nERROR rejoining the network (%d) ... aborting!\n",ret);
            /* Wait for watchdog reset */
            sleep_until_reset(WDTO_8S);
        } else {
            printf(" connected\n");
        }
#if ENABLE_DBG_MSG
        /* Print the contents of the message to be sent */
        dbg_print_msg(&msg);
#endif
#if ZIGBEE_ENABLE_ACK
        uint8_t fid = fid_get_next();
        ret = xbee_transmit_unicast(SEN_MSG_MAC_CH, (uint8_t*)&msg, sizeof(MSG_t), fid);
        if(ret == XBEE_RET_OK) {
            printf("%d. sensor value update sent ... ",msg.time);
            /* Check the transmit response */
            uint16_t addr=0x0000;
            uint8_t retries=0, status=0, discovery=0, fid_ret=0;
            ret = xbee_transmit_status_ext(&addr, &retries, &status, &discovery, &fid_ret);
            if((fid == fid_ret) && (ret == XBEE_RET_OK)) {
                if(status == XBEE_TRANSMIT_STAT_DEL_OK) {
                    printf("positive response received!\n\n");
                    x_ic_dec(X_IC_DEC_NORM);
                } else {
                    printf("NEGATIVE response received (%d)!\n\n",status);
                    x_ic_inc(X_IC_INC_SERIOUS);
                }
            } else {
                printf("ERROR receiving response (%d)!\n\n",ret);
                x_ic_inc(X_IC_INC_SERIOUS);
            }
        } else {
            printf("ERROR sending message (%d)!\n\n",ret);
            x_ic_inc(X_IC_INC_SERIOUS);
        }
#else
        /* Send the measurement to the CH (without response) */
        ret = xbee_transmit_unicast(SEN_MSG_MAC_CH, (uint8_t*)&msg, sizeof(MSG_t), 0);
        if(ret == XBEE_RET_OK) {
            printf("%d. sensor value update sent\n\n",msg.time);
            x_ic_dec(X_IC_DEC_NORM);
        } else {
            printf("ERROR sending message (%d)!\n\n",ret);
            x_ic_inc(X_IC_INC_SERIOUS);
        }
#endif
        /* Increment message number ("time") */
        msg.time++;


        /*** 3.6) disable modules/sensors *****************************/
        /* Stop timer1 to save runtime measurement */
        timer1_stop();
        /* Save timer1 counter value */
        runtime = timer1_get_tcnt();
        /* Reset timer1 counter value to 0 */
        timer1_set_tcnt(0);
        /* Send xbee to sleep */
        ret = xbee_sleep_enable();
        if(ret != XBEE_RET_OK) {
            printf("Couldn't send xbee radio to sleep (%d) ... aborting!\n",ret);
            /* Wait for watchdog reset */
            sleep_until_reset(WDTO_8S);
        }
        /* Disable ADC */
        adc_disable();
        /* Disable the self-diagnostics */
        diag_disable();


#if ASNX_VERSION_MINOR>0
        /*** 3.7) put MCU to sleep ************************************/
        sleep_enable();
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
    /* Give the MCU some time to fully wake up */
    _delay_ms(5);
}
#endif
