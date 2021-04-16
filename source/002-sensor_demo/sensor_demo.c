/****
 * @brief   Demo application for a set of different sensors.
 *
 * Simple demo application to read the measurements of several sensors
 * (e.g., thermistor via ADC, AM2302 and DS18X20 via OWI, and LM75
 * and STEMMA SOIL via TWI) and print the values every 15 seconds
 * (controlled via the systick handler) via UART.
 *
 * @file    /002-sensor_demo/sensor_demo.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1 $
 * @date    $Date: 2021/04/16 $
 *****/

/*** DEMO CONFIGURATION ***/
/* Update interval [s] */
#define UPDATE_INTERVAL     15
/* Enable/disable sensors */
#define ENABLE_JT103        (1)     /**< Enable the 103JT thermistor (via ADC) */
#define ENABLE_DS18X20      (1)     /**< Enable the DS18X20 sensor (OWI) */
#define ENABLE_AM2302       (1)     /**< Enable the AM2302 sensor (OWI) */
#define ENABLE_BME280       (1)     /**< Enable the BME280 sensor (TWI) */
#define ENABLE_TMP275       (1)     /**< Enable the TMP275 sensor (TWI) */
#define ENABLE_LM75         (1)     /**< Enable the LM75 sensor (TWI) */
#define ENABLE_STEMMA       (1)     /**< Enable the STEMMA SOIL sensor (TWI) */


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
/* Sensors */
#if ENABLE_JT103
#  include "sensors/jt103.h"
#endif
#if ENABLE_DS18X20
#  include "sensors/ds18x20.h"
DS18X20_t ds18b20;
#endif
#if ENABLE_AM2302
#  include "sensors/dht.h"
DHT_t am2302;
#endif
#if ENABLE_BME280
#  include "sensors/bme280.h"
BME280_t bme280;
#endif
#if ENABLE_TMP275
#  include "sensors/tmp275.h"
TMP275_t tmp275;
#endif
#if ENABLE_LM75
#  include "sensors/lm75.h"
LM75_t lm75;
#endif
#if ENABLE_STEMMA
#  include "sensors/stemma_soil.h"
STEMMA_t stemma;
#endif


/***** FUNCTION CALLBACK **********************************************/
/***
 * Function to be called by the systick callback to read sensor values
 * and print them via UART.
 ***/
void read_and_print(void) {
#if (ENABLE_DS18X20 || ENABLE_AM2302 || ENABLE_BME280 || ENABLE_TMP275 || ENABLE_LM75 || ENABLE_STEMMA)
    float f_tmp;
#endif
#if ENABLE_STEMMA
    uint16_t u_tmp;
#endif
    
    /*** internal ADC ***/
    printf("=> Internal ADC\n");
    printf("...Supply voltage: %1.2f V\n",adc_read_vcc());
    
#if ENABLE_JT103
    /*** 103JT thermistor (via ADC) ***/
    printf("=> 103JT thermistor\n");
    printf("...Temperature: %2.2f C\n",jt103_get_temperature(adc_read()));
#endif

#if ENABLE_DS18X20
    /*** DS18B20 ***/
    printf("=> DS18B20\n");
    printf("...Temperature: ");
    if(ds18x20_get_temperature(&ds18b20, &f_tmp) == DS18X20_RET_OK) {
        printf("%2.2f C\n",f_tmp);
    } else {
        printf("ERROR\n");
    }
#endif

#if ENABLE_AM2302
    /*** AM2302 ***/
    printf("=> AM2302\n");
    printf("...Temperature: ");
    if(dht_get_temperature(&am2302, &f_tmp) == DHT_RET_OK) {
        printf("%2.2f C\n",f_tmp);
    } else {
        printf("ERROR\n");
    }
    printf("...Humidity: ");
    if(dht_get_humidity(&am2302, &f_tmp) == DHT_RET_OK) {
        printf("%2.2f %%\n",f_tmp);
    } else {
        printf("ERROR\n");
    }
#endif

#if ENABLE_BME280
    /*** BME280 ***/
    printf("=> BME280\n");
    printf("...Temperature: ");
    if(bme280_get_temperature(&bme280, &f_tmp) == BME280_RET_OK) {
        printf("%2.2f C\n",f_tmp);
    } else {
        printf("ERROR\n");
    }
    printf("...Humidity: ");
    if(bme280_get_humidity(&bme280, &f_tmp) == BME280_RET_OK) {
        printf("%2.2f %%\n",f_tmp);
    } else {
        printf("ERROR\n");
    }
    printf("...Pressure: ");
    if(bme280_get_pressure(&bme280, &f_tmp) == BME280_RET_OK) {
        printf("%2.2f hPa\n",f_tmp);
    } else {
        printf("ERROR\n");
    }
#endif

#if ENABLE_TMP275
    /*** TMP275 ***/
    printf("=> TMP275\n");
    printf("...Temperature: ");
    if(tmp275_get_temperature(&tmp275, &f_tmp) == TMP275_RET_OK) {
        printf("%2.2f C\n",f_tmp);
    } else {
        printf("ERROR\n");
    }
#endif

#if ENABLE_LM75
    /*** LM75 ***/
    printf("=> LM75\n");
    printf("...Temperature: ");
    if(lm75_get_temperature(&lm75, &f_tmp) == LM75_RET_OK) {
        printf("%2.2f C\n",f_tmp);
    } else {
        printf("ERROR\n");
    }
#endif

#if ENABLE_STEMMA
    /*** STEMMA SOIL ***/
    printf("=> STEMMA SOIL\n");
    printf("...Temperature: ");
    if(stemma_get_temperature(&stemma, &f_tmp) == STEMMA_RET_OK) {
        printf("%2.2f C\n",f_tmp);
    } else {
        printf("ERROR\n");
    }
    printf("...Capacity: ");
    if(stemma_get_capacity(&stemma, &u_tmp) == STEMMA_RET_OK) {
        printf("%4d\n",u_tmp);
    } else {
        printf("ERROR\n");
    }
#endif
    
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
    
    /* Initialize the UART0 */
    uart1_init();
    /* Initialize the printf function to use the uart1_putc() function for output */
    printf_init(uart1_putc);
    
    /* Initialize the ADC */
    adc_init(ADC_ADPS_16,ADC_REFS_VCC);

#if ENABLE_DS18X20
    /* Initialize the DS18B20 sensor */
    if(ds18x20_init(&ds18b20, &DDRD, &PORTD, &PIND, PD6) != DS18X20_RET_OK) {
        printf("Couldn't initialize DS18B20 ... aborting!\n");
        while(1);
    }
#endif

#if ENABLE_AM2302
    /* Initialize the AMS2302 sensor */
    dht_init(&am2302, &DDRD, &PORTD, &PIND, PD7, DHT_DEV_AM2302);
#endif

#if ENABLE_BME280
    /* Initialize the BME280 sensor */
    if(bme280_init(&bme280, BME280_I2C_ADDRESS) != BME280_RET_OK) {
        printf("Couldn't initialize BME280 ... aborting!\n");
        while(1);
    }
#endif

#if ENABLE_TMP275
    /* Initialize the TMP275 sensor */
    if(tmp275_init(&tmp275, TMP275_I2C_ADDRESS) != TMP275_RET_OK) {
        printf("Couldn't initialize TMP275 ... aborting!\n");
        while(1);
    }
#endif

#if ENABLE_LM75
    /* Initialize the LM75 sensor */
    if(lm75_init(&lm75, (LM75_I2C_ADDRESS | LM75_I2C_ADDRESS_A0)) != LM75_RET_OK) {
        printf("Couldn't initialize LM75 ... aborting!\n");
        while(1);
    }
#endif

#if ENABLE_STEMMA
    /* Initialize the STEMMA SOIL sensor */
    if(stemma_init(&stemma, STEMMA_I2C_ADDRESS) != STEMMA_RET_OK) {
        printf("Couldn't initialize STEMMA ... aborting!\n");
        while(1);
    }
#endif

    /* Initialize the systick timer */
    systick_init();
    /* Set a systick callback function to be called every second */
    systick_set_callback_sec(update);
    
    /*** Enable interrupts globally ***/
    sei();
    
    /* Print welcome message */
    printf("=== STARTING UP ... ===\n");
    
    /* Main routine actually does nothing ...
     * everything happens as callback from the ISR */
    while (1);

    return(0);
}
