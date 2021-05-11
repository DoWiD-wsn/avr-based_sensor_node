/* Update interval [min] */
#define UPDATE_INTERVAL         (10)


/***** INCLUDES *******************************************************/
/*** AVR ***/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
/*** ASNX LIB ***/
/* MCU */
#include "adc/adc.h"
#include "hw/led.h"
#include "uart/uart.h"
/* Sensors */
#include "sensors/tmp275.h"
/* Misc */
#include "util/printf.h"

/* To test */
#include "rtc/pcf85263.h"


/***** GLOBAL VARIABLES ***********************************************/
/* Variable (flag) for barrier synchronization */
uint8_t barrier = 1;


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
    /* Temporary variable for sensor measurements */
    float measurement = 0.0, vcc = 0.0;
    uint16_t adcres = 0;
    /* Device handles */
    TMP275_t tmp275;
    
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
    
    /* Configure INT2 to fire interrupt when logic level is "low" */
    EICRA = 0x00;
    EIMSK = _BV(INT2);
    
    /* Give the hardware time to start up */
    led_show_startup();
    
    /* Print welcome message */
    printf("=== STARTING UP ... ===\n");

    /* Initialize the TMP275 sensor */
    if(tmp275_init(&tmp275, TMP275_I2C_ADDRESS) != TMP275_RET_OK) {
        printf("Couldn't initialize TMP275!\n");
    } else {
        printf("... TMP275 ready\n");
    }
    
    /* Enable interrupts globally */
    sei();
    
    /* Main routine ... */
    while(1) {
        /*** Barrier synchronization */
        /* Wait until the barrier sync flag is set */
        while(barrier == 0) {
            _delay_ms(100);
        }
        /* Reset barrier sync flag */
        barrier = 0;
        
        /*** ADC self-diagnosis (via ADC CH0) ***/
        /* Constant voltage divider (1:1) */
        printf("... ADC self-diagnosis: %d\n", adc_read_input(ADC_CH0));
        
        /*** MCU supply voltage (via ADC) ***/
        /* Supply voltage in volts (V) */
        vcc = adc_read_vcc();
        printf("... Supply voltage: %.2f\n", vcc);
        
        /*** Battery voltage (via ADC) ***/
        /* Supply voltage in volts (V) */
        adcres = adc_read_input(ADC_CH2);
        /* Calculate voltage from value (voltage divider 1:1) */
        measurement = 2.0 * (adcres * (vcc / 1023.0));
        printf("... Battery voltage: %.2f\n", measurement);

        /*** TMP275 ***/
        /* Temperature in degree Celsius (°C) */
        if(tmp275_get_temperature(&tmp275, &measurement) == TMP275_RET_OK) {
            printf("... TMP275 temperature: %.2f\n", measurement);
        } else {
            printf("... TMP275 temperature: FAILED!\n");
        }

        printf("\n");
    }

    return 0;
}


/***** ISR ************************************************************/
ISR(INT2_vect) {
    led2_toggle();
}
