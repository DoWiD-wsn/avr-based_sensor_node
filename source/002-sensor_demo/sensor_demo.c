/****
 * @brief   Demo application for the ASN(x) sensors libraries.
 *
 * Simple demo application to read the measurements of several sensors
 * (e.g., thermistor via ADC, AM2302 via OWI, and LM75 via TWI) and
 * print the values every few seconds (controlled by the systick
 * handler) via UART.
 *
 * @file    /002-sensor_demo/sensor_demo.c
 * @author  Dominik Widhalm
 * @version 1.3.1
 * @date    2022/04/13
 *****/

/*** DEMO CONFIGURATION ***/
/* Update interval [s] */
#define UPDATE_INTERVAL     10
/* Enable/disable sensors */
#define ENABLE_JT103        (1)     /**< Enable the 103JT thermistor (via ADC) */
#define ENABLE_TMP275       (1)     /**< Enable the TMP275 sensor (TWI) */
#define ENABLE_DS18X20      (0)     /**< Enable the DS18X20 sensor (OWI) */
#define ENABLE_AM2302       (0)     /**< Enable the AM2302 sensor (OWI) */
#define ENABLE_BH1750       (1)     /**< Enable the BH1750 sensor (TWI) */
#define ENABLE_BME280       (0)     /**< Enable the BME280 sensor (TWI) */
#define ENABLE_LM75         (0)     /**< Enable the LM75 sensor (TWI) */
#define ENABLE_SHT30        (1)     /**< Enable the SHT30 sensor (TWI) */
#define ENABLE_SHTC3        (0)     /**< Enable the SHTC3 sensor (TWI) */
#define ENABLE_STEMMA       (0)     /**< Enable the STEMMA SOIL sensor (TWI) */
#define ENABLE_ZMOD4410     (1)     /**< Enable the ZMOD4410 sensor (TWI) */


/***** INCLUDES *******************************************************/
/*** AVR ***/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
/*** ASNX LIB ***/
#include "adc/adc.h"
#include "hw/led.h"
#include "timer/systick.h"
#include "uart/uart.h"
#include "util/printf.h"
#include "util/diagnostics.h"
/* Sensors */
// JT103
#if ENABLE_JT103
#  include "sensors/jt103.h"
#endif
// TMP275
#if ENABLE_TMP275
#  include "sensors/tmp275.h"
TMP275_t tmp275;
#endif
// DS18X20
#if ENABLE_DS18X20
#  include "sensors/ds18x20.h"
DS18X20_t ds18b20;
#endif
// AM2302
#if ENABLE_AM2302
#  include "sensors/dht.h"
DHT_t am2302;
#endif
// BH1750
#if ENABLE_BH1750
#  include "sensors/bh1750.h"
BH1750_t bh1750;
#endif
// BME280
#if ENABLE_BME280
#  include "sensors/bme280.h"
BME280_t bme280;
#endif
// LM75
#if ENABLE_LM75
#  include "sensors/lm75.h"
LM75_t lm75;
#endif
// SHT30
#if ENABLE_SHT30
#  include "sensors/sht30.h"
SHT30_t sht30;
#endif
// SHTC3
#if ENABLE_SHTC3
#  include "sensors/shtc3.h"
SHTC3_t shtc3;
#endif
// STEMMA
#if ENABLE_STEMMA
#  include "sensors/stemma_soil.h"
STEMMA_t stemma;
#endif
// ZMOD4410
#if ENABLE_ZMOD4410
#  include "sensors/zmod4410.h"
ZMOD4410_t zmod4410;
#endif


/***** LOCAL FUNCTION PROTOTYPES **************************************/
void error_state(void);
void read_and_print(void);
void update(void);


/***** FUNCTIONS ******************************************************/
/***
 * Function to be called in case of errors. It lights both LEDs and
 * enters an endless loop.
 ***/
void error_state(void) {
    /* Turn on both LEDs */
    led1_low();
    led2_low();
    /* Enter endless loop */
    while(1);
}


/***
 * Function to be called by the systick callback to read sensor values
 * and print them via UART.
 ***/
void read_and_print(void) {
    /*** internal ADC ***/
    printf("=> Internal ADC\n");
    printf("   ... Supply voltage: %1.2f V\n",adc_read_vcc());


#if ENABLE_JT103
    /*** 103JT thermistor ***/
    printf("=> 103JT thermistor (ADC)\n");
    printf("   ... Temperature: %2.2f C\n",jt103_get_temperature(adc_read()));
#endif


#if ENABLE_TMP275
    /*** TMP275 ***/
    printf("=> TMP275 (TWI)\n");
    printf("   ... Temperature: ");
    float tmp275_tmp;
    if(tmp275_get_temperature(&tmp275, &tmp275_tmp) == TMP275_RET_OK) {
        printf("%.2f C\n",tmp275_tmp);
    } else {
        printf("ERROR\n");
    }
#endif


#if ENABLE_DS18X20
    /*** DS18B20 ***/
    printf("=> DS18B20 (OWI)\n");
    printf("   ... Temperature: ");
    float ds18b20_tmp;
    if(ds18x20_get_temperature(&ds18b20, &ds18b20_tmp) == DS18X20_RET_OK) {
        printf("%.2f C\n",ds18b20_tmp);
    } else {
        printf("ERROR\n");
    }
#endif


#if ENABLE_AM2302
    /*** AM2302 ***/
    printf("=> AM2302 (OWI)\n");
    float am2302_tmp_t, am2302_tmp_h;
    if(dht_get_temperature_humidity(&am2302, &am2302_tmp_t, &am2302_tmp_h) == DHT_RET_OK) {
        printf("   ... Temperature: %2.2f C\n",am2302_tmp_t);
        printf("   ... Humidity: %2.2f %%\n",am2302_tmp_h);
    } else {
        printf("   ... ERROR\n");
    }
#endif


#if ENABLE_BH1750
    /*** BH1750 ***/
    printf("=> BH1750 (TWI)\n");
    printf("   ... Illuminance: ");
    uint16_t bh1750_tmp;
    if(bh1750_get_illuminance(&bh1750, &bh1750_tmp) == BH1750_RET_OK) {
        printf("%d lx\n",bh1750_tmp);
    } else {
        printf("ERROR\n");
    }
#endif


#if ENABLE_BME280
    /*** BME280 ***/
    printf("=> BME280 (TWI)\n");
    float bme280_tmp;
    printf("   ... Temperature: ");
    if(bme280_get_temperature(&bme280, &bme280_tmp) == BME280_RET_OK) {
        printf("%.2f C\n",bme280_tmp);
    } else {
        printf("ERROR\n");
    }
    printf("   ... Humidity: ");
    if(bme280_get_humidity(&bme280, &bme280_tmp) == BME280_RET_OK) {
        printf("%.2f %%\n",bme280_tmp);
    } else {
        printf("ERROR\n");
    }
    printf("   ... Pressure: ");
    if(bme280_get_pressure(&bme280, &bme280_tmp) == BME280_RET_OK) {
        printf("%.2f hPa\n",bme280_tmp);
    } else {
        printf("ERROR\n");
    }
#endif


#if ENABLE_LM75
    /*** LM75 ***/
    printf("=> LM75 (TWI)\n");
    printf("   ... Temperature: ");
    float lm75_tmp;
    if(lm75_get_temperature(&lm75, &lm75_tmp) == LM75_RET_OK) {
        printf("%.2f C\n",lm75_tmp);
    } else {
        printf("ERROR\n");
    }
#endif


#if ENABLE_SHT30
    /*** SHT30 ***/
    printf("=> SHT30 (TWI)\n");
    printf("   ... Temperature: ");
    float sht30_tmp;
    if(sht30_get_temperature(&sht30, &sht30_tmp) == SHT30_RET_OK) {
        printf("%.2f C\n",sht30_tmp);
    } else {
        printf("ERROR\n");
    }
    printf("   ... Humidity: ");
    if(sht30_get_humidity(&sht30, &sht30_tmp) == SHT30_RET_OK) {
        printf("%.2f %%\n",sht30_tmp);
    } else {
        printf("ERROR\n");
    }
#endif


#if ENABLE_SHTC3
    /*** SHTC3 ***/
    printf("=> SHTC3 (TWI)\n");
    printf("   ... Temperature: ");
    float shtc3_tmp;
    if(shtc3_get_temperature(&shtc3, &shtc3_tmp, 1) == SHTC3_RET_OK) {
        printf("%.2f C\n",shtc3_tmp);
    } else {
        printf("ERROR\n");
    }
    printf("   ... Humidity: ");
    if(shtc3_get_humidity(&shtc3, &shtc3_tmp, 1) == SHTC3_RET_OK) {
        printf("%.2f %%\n",shtc3_tmp);
    } else {
        printf("ERROR\n");
    }
#endif


#if ENABLE_STEMMA
    /*** STEMMA SOIL ***/
    printf("=> STEMMA SOIL (TWI)\n");
    printf("   ... Temperature: ");
    float stemma_tmp;
    if(stemma_get_temperature(&stemma, &stemma_tmp) == STEMMA_RET_OK) {
        printf("%.2f C\n",stemma_tmp);
    } else {
        printf("ERROR\n");
    }
    printf("   ... Humidity: ");
    if(stemma_get_humidity(&stemma, &stemma_tmp) == STEMMA_RET_OK) {
        printf("%.2f %%\n",stemma_tmp);
    } else {
        printf("ERROR\n");
    }
#endif


#if ENABLE_ZMOD4410
    /*** ZMOD4410 ***/
    printf("=> ZMOD4410 (TWI)\n");
    ZMOD4410_DATA_t zmod4410_tmp;
    if(zmod4410_get_measurement(&zmod4410, &zmod4410_tmp) == ZMOD4410_RET_OK) {
        printf("   ... CDA : %.2f (log10)\n",zmod4410_tmp.log_rcda);
        printf("   ... IAQ : %.2f\n",zmod4410_tmp.iaq);
        printf("   ... TVOC: %.2f mg/m^3\n",zmod4410_tmp.tvoc);
        printf("   ... EtOH: %.2f ppm\n",zmod4410_tmp.etoh);
        printf("   ... eCO2: %.2f ppm\n",zmod4410_tmp.eco2);
    } else {
        printf("   ... ERROR\n");
    }
#endif


    /* End of cycle */
    printf("\n");
}


/***
 * Callback function to be called by the systick timer.
 ***/
void update(void) {
    static int cnt = 0;
    /* Toggle LED1 */
    led1_toggle();
    /* Check if 10 seconds have elapsed */
    if(++cnt == UPDATE_INTERVAL) {
        /* Turn LED2 on */
        led2_low();
        /* Call read'n'print function */
        read_and_print();
        /* Turn LED2 off */
        led2_high();
        /* Reset counter */
        cnt = 0;
    }
}


/***** MAIN ***********************************************************/
int main(void) {
    /*** Initialize the hardware ***/
    /* Initialize the user LEDs (LED1 to low and LED2 to high) */
    led_init();
    led1_low();
    led2_high();
    
    /* Initialize I2C master interface */
    i2c_init();
    
    /* Initialize the UART0 */
    uart1_init(9600UL);
    /* Initialize the printf function to use the uart1_write_char() function for output */
    printf_init(uart1_write_char);
    
    /* Initialize the ADC */
    adc_init(ADC_ADPS_32,ADC_REFS_VCC);
    adc_disable_din(0x07);

    /* Initialize the diagnostic circuitry */
    diag_init();
    diag_enable();

    /* Print welcome message */
    printf("=== STARTING UP ... ===\n");

#if ENABLE_TMP275
    /* Initialize the TMP275 sensor */
    if(tmp275_init(&tmp275, TMP275_I2C_ADDRESS) != TMP275_RET_OK) {
        printf("Couldn't initialize TMP275 ... aborting!\n");
        error_state();
    }
#endif

#if ENABLE_DS18X20
    /* Initialize the DS18B20 sensor */
    if(ds18x20_init(&ds18b20, &DDRD, &PORTD, &PIND, PD6) != DS18X20_RET_OK) {
        printf("Couldn't initialize DS18B20 ... aborting!\n");
        error_state();
    }
#endif

#if ENABLE_AM2302
    /* Initialize the AMS2302 sensor */
    if(dht_init(&am2302, &DDRD, &PORTD, &PIND, PD7, DHT_DEV_AM2302) != DHT_RET_OK) {
        printf("Couldn't initialize AMS2302 ... aborting!\n");
        error_state();
    }
#endif

#if ENABLE_BH1750
    /* Initialize the BH1750 sensor */
    if(bh1750_init_default(&bh1750) != BH1750_RET_OK) {
        printf("Couldn't initialize BH1750 ... aborting!\n");
        error_state();
    }
#endif

#if ENABLE_BME280
    /* Initialize the BME280 sensor */
    if(bme280_init(&bme280, BME280_I2C_ADDRESS) != BME280_RET_OK) {
        printf("Couldn't initialize BME280 ... aborting!\n");
        error_state();
    }
#endif

#if ENABLE_LM75
    /* Initialize the LM75 sensor */
    if(lm75_init(&lm75, (LM75_I2C_ADDRESS | LM75_I2C_ADDRESS_A0)) != LM75_RET_OK) {
        printf("Couldn't initialize LM75 ... aborting!\n");
        error_state();
    }
#endif

#if ENABLE_SHT30
    /* Initialize the SHT30 sensor */
    if(sht30_init(&sht30, SHT30_I2C_ADDRESS) != SHT30_RET_OK) {
        printf("Couldn't initialize SHT30 ... aborting!\n");
        error_state();
    }
#endif

#if ENABLE_SHTC3
    /* Initialize the SHTC3 sensor */
    if(shtc3_init(&shtc3, SHTC3_I2C_ADDRESS) != SHTC3_RET_OK) {
        printf("Couldn't initialize SHTC3 ... aborting!\n");
        error_state();
    }
#endif

#if ENABLE_STEMMA
    /* Initialize the STEMMA SOIL sensor */
    if(stemma_init(&stemma, STEMMA_I2C_ADDRESS) != STEMMA_RET_OK) {
        printf("Couldn't initialize STEMMA ... aborting!\n");
        error_state();
    }
#endif

#if ENABLE_ZMOD4410
    /* Initialize the ZMOD4410 sensor */
    if(zmod4410_init(&zmod4410, ZMOD4410_I2C_ADDRESS) != ZMOD4410_RET_OK) {
        printf("Couldn't initialize ZMOD4410 ... aborting!\n");
        error_state();
    }
#endif

    /* Initialize the systick timer */
    systick_init();
    /* Set a systick callback function to be called every second */
    systick_set_callback_sec(update);
    
    /*** Enable interrupts globally ***/
    sei();
    
    /* Main routine actually does nothing ...
     * everything happens as callback from the ISR */
    while(1);
    
    /* This point should never been reached */
    return(0);
}
