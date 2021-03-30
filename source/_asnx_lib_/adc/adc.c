/**
 *  Source file for AVR ADC functionality.
 */

/***** INCLUDES ***************************************************************/
#include "adc.h"
#include <avr/io.h>
#include <util/delay.h>


/***** GLOBAL VARIABLES *******************************************************/
/* Use precalculated temperature compensation: *
 * (0)  ... use precalculated correction coefficients
 * (1)  ... use signature calculation values (not correct!?)
 */
#define ADC_TEMP_CORRECTION     (0)


/***** FUNCTION PROTOTYPES ****************************************************/
void adc_dummy_conversion(void);


/***** FUNCTIONS **************************************************************/
/*** GENERAL (DE)INIT *************************************************/
/*
 * Initialization of the ADC
 */
void adc_init(uint8_t prescaler, uint8_t reference) {
    /* Select the input channel initially to CH0 */
    adc_set_channel(ADC_CH0);
    /* Select the prescaler */
    adc_set_prescaler(prescaler);
    /* Select the reference voltage */
    adc_set_reference(reference);
    /* Enable the ADC */
    adc_enable();
    /* Perform a dummy conversion */
    adc_dummy_conversion();
}


/*
 * De-initialization of the ADC
 */
void adc_deinit(void) {
    /* Clear the ADC's internal multiplexer */
    ADMUX = 0x00;
    /* Clear the control register */
    ADCSRA = 0x00;
}


/*** SPECIFIC INIT ****************************************************/
/*
 * Enable the ADC peripheral
 */
void adc_enable(void) {
    /* Enable the ADC */
    ADCSRA |= _BV(ADEN);
}


/*
 * Disable the ADC peripheral
 */
void adc_disable(void) {
    /* Disable the ADC */
    ADCSRA &= ~_BV(ADEN);
}


/*
 * Set the ADC channel
 */
void adc_set_channel(uint8_t channel) {
     /* Select desired channel with internal multiplexer */
    ADMUX = ((ADMUX & 0xE0) | (channel & 0x0F));
}


/*
 * Set the ADC prescaler
 */
void adc_set_prescaler(uint8_t prescaler) {
    /* Select desired prescaler */
    ADCSRA = ((ADCSRA & 0xF8) | (prescaler & 0x07));
}


/*
 * Set the ADC reference
 */
void adc_set_reference(uint8_t reference) {
    /* Select desired reference */
    ADMUX = ((ADMUX & 0x2F) | ((reference & 0x03) << 6));
}


/*** READING **********************************************************/

/*
 * Read the current channel of the ADC unit
 */
uint16_t adc_read(void) {
    /* Start a single conversion */
    ADCSRA |= _BV(ADSC);
    /* Wait for conversion complete */
    while(!(ADCSRA & _BV(ADIF)));
    /* Clear ADIF */
    ADCSRA |= _BV(ADIF);
    /* Return the converted value */
    return ADCW;
}


/*
 * Read the given channel of the ADC unit
 */
uint16_t adc_read_channel(uint8_t channel) {
    /* Set the given channel */
    adc_set_channel(channel);
    /* Return the conversion result */
    return adc_read();
}


/*
 * Perform a dummy conversion
 */
void adc_dummy_conversion(void) {
    /* Perform a conversion but neglect result */
    (void)adc_read();
}


/*** SPECIAL FUNCTIONS ************************************************/
/*
 * Measure the supply voltage
 */
float adc_read_vss(void) {
    /* Set the register accordingly */
    ADMUX  = 0x4E;
    /* Give the reference some time to settle */
    _delay_ms(ADC_DELAY_CHANGE_REFERENCE);
    /* Perform a dummy conversion */
    adc_dummy_conversion();
    /* Return the converted ADC value */
    return (1.1 * (1023.0/(float)adc_read()));
}


/*
 * Measure the core temperature
 */
float adc_read_temp(void) {
    /* Set the register accordingly */
    ADMUX  = 0xC8;
    /* Give the reference some time to settle */
    _delay_ms(ADC_DELAY_CHANGE_REFERENCE);
    /* Perform a dummy conversion */
    adc_dummy_conversion();
    /* Return the converted ADC value */
    return adc_convert_temperature(adc_read());
}


/*** MISC *************************************************************/
/*
 * Convert temperature reading to degree Celsius
 */
float adc_convert_temperature(uint16_t input) {
#if (ADC_TEMP_CORRECTION==1)
    #include <avr/boot.h>
    /* Get the calibration values (see datasheet Table 26-5) */
    float offset = boot_signature_byte_get(0x0002);
    float gain = boot_signature_byte_get(0x0003);
    /* Convert reading to degree Celsius (see datasheet 23.8.1) */
    float temp = ((((float)input - (273.0 + 100.0 - offset)) * 128.0) / gain) + 25.0;
    /* Return the result */
    return (float)(temp);
#else
    // https://playground.arduino.cc/Main/InternalTemperatureSensor/
    return ((float)input - 324.31) / 1.22;
#endif
}
