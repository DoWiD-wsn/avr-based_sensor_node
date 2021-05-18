/* Update interval [min] */
#define UPDATE_INTERVAL         (10)


/***** INCLUDES *******************************************************/
/*** AVR ***/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
/*** ASNX LIB ***/
/* MCU */
#include "adc/adc.h"
#include "i2c/i2c.h"
#include "hw/led.h"
#include "uart/uart.h"
/* RTC */
#include "rtc/pcf85263.h"
/* Sensors */
#include "sensors/tmp275.h"
/* Misc */
#include "util/printf.h"


/***** MAIN ***********************************************************/
int main(void) {
    /*** Local variables ***/
    /* Temporary variable for sensor measurements */
    uint8_t reg = 0;
    /* Date/time structure */
    PCF85263_CNTTIME_t time = {0};
    
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
    
    /*** Setup the RTC as wake-up source for MCU ***/
    /* Initialize the RTC */
    if(pcf85263_init() != PCF85263_RET_OK) {
        printf("Couldn't initialize RTC ... aborting!\n");
        while(1);
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
    /* Enable stop-watch mode (read first to get 100TH and STOPM bits) */
    if(pcf85263_get_function(&reg) != PCF85263_RET_OK) {
        printf("RTC: Function configuration read FAILED ... aborting!\n");
        while(1);
    }
    reg |= PCF85263_CTL_FUNC_RTCM;
    if(pcf85263_set_function(reg) != PCF85263_RET_OK) {
        printf("RTC: Function configuration write FAILED ... aborting!\n");
        while(1);
    }
    /* Set desired wake-up time */
    time.minutes = 2;
    if(pcf85263_set_stw_alarm1(&time) != PCF85263_RET_OK) {
        printf("RTC: Alarm time configuration FAILED ... aborting!\n");
        while(1);
    }
    /* Enable the alarm */
    if(pcf85263_set_stw_alarm_enables(PCF85263_RTC_ALARM_MIN_A1E) != PCF85263_RET_OK) {
        printf("RTC: Alarm enable configuration FAILED ... aborting!\n");
        while(1);
    }
    /* Enable the alarm interrupt */
    if(pcf85263_set_inta_en(PCF85263_CTL_INTA_A1IEA) != PCF85263_RET_OK) {
        printf("RTC: Alarm enable configuration FAILED ... aborting!\n");
        while(1);
    }
    /* Start RTC */
    if(pcf85263_start() != PCF85263_RET_OK) {
        printf("Couldn't start RTC ... aborting!\n");
        while(1);
    } else {
        printf("... RTC started\n");
    }
    /**********************************/
    
    /* Configure INT2 to fire interrupt when logic level is "low" */
    EICRA = 0x00;
    EIMSK = _BV(INT2);
    
    /* Configure the sleep mode */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    
    /* Print welcome message */
    printf("=== STARTING UP ... ===\n");
    
    /* Enable interrupts globally */
    sei();
    
    /* Main routine ... */
    while(1) {
        /* Enter power down mode */
        sleep_mode();
        /* ... everything else happens in the ISR */
    }

    return 0;
}


/***** ISR ************************************************************/
ISR(INT2_vect) {
    /* Reset the TWI */
    i2c_reset();
    /* Enable ADC */
    adc_enable();
    /* Reset stop-watch time */
    PCF85263_CNTTIME_t time = {0};
    pcf85263_set_stw_time(&time);
    
    /*** Update Sensor Values ***/
    printf("Here I am ...\n");
    /****************************/
    
    /* Disable ADC */
    adc_disable();
}
