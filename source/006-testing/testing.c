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
    /* Date/time structure */
    PCF85263_DATETIME_t time;
    
    /*** Initialize the hardware ***/
    /* Initialize the user LEDs and disable both by default */
    led_init();
    led1_high();
    led2_high();
    
    /* Initialize I2C master interface */
    i2c_init();
    
    /* Initialize the ADC */
    adc_init(ADC_ADPS_16,ADC_REFS_VCC);

    /* Initialize UART1 for debug purposes */
    uart1_init();
    /* Initialize the printf function to use the uart1_putc() function for output */
    printf_init(uart1_putc);
    
    /* Initialize the RTC */
    if(pcf85263_init() != PCF85263_RET_OK) {
        printf("Couldn't initialize RTC ... aborting!\n");
        while(1);
    } else {
        printf("... RTC ready\n");
    }
    /* Disable the battery switch */
    if(pcf85263_set_batteryswitch(PCF85263_CTL_BATTERY_BSOFF) != PCF85263_RET_OK) {
        printf("RTC: Battery switch configuration FAILED ... aborting!\n");
        while(1);
    }
    /* Disable CLK pin; INTA output */
    if(pcf85263_set_pin_io(PCF85263_CTL_CLKPM | PCF85263_CTL_INTAPM_INTA) != PCF85263_RET_OK) {
        printf("RTC: Battery switch configuration FAILED ... aborting!\n");
        while(1);
    }
    /* Set RTC date/time */
#if PCF85263_100TH_SECONDS_ENABLE==1
    time.msec10 = 0;
#endif
    time.seconds = 0;
    time.minutes = 0;
    time.hours = 0;
    time.days = 10;
    time.wday = 0;
    time.months = 5;
    time.years = 21;
    if(pcf85263_set_rtc_datetime(&time) != PCF85263_RET_OK) {
        printf("RTC set date/time FAILED ... aborting!\n");
        while(1);
    }
    /* Start RTC */
    if(pcf85263_start() != PCF85263_RET_OK) {
        printf("Couldn't start RTC ... aborting!\n");
        while(1);
    } else {
        printf("... RTC started\n");
    }
    
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
        
        /*** RTC ***/
        if(pcf85263_get_rtc_datetime(&time) != PCF85263_RET_OK) {
            printf("RTC read date/time FAILED!\n");
        } else {
#if PCF85263_100TH_SECONDS_ENABLE==1
            printf("... RTC: %02d.%02d.20%02d - %02d:%02d:%02d.%02d0\n",time.days,time.months,time.years,time.hours,time.minutes,time.seconds,time.msec10);
#else
            printf("... RTC: %02d.%02d.20%02d - %02d:%02d:%02d\n",time.days,time.months,time.years,time.hours,time.minutes,time.seconds);
#endif
        }

        printf("\n");
        _delay_ms(5000);
    }

    return 0;
}


/***** ISR ************************************************************/
ISR(INT2_vect) {
    led2_toggle();
}
