/*****
 * @brief   ASN(x) ADC library
 *
 * Library to support the use of the ADC.
 *
 * @file    /_asnx_lib_/adc/adc.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/13 $
 *****/

/***** INCLUDES *******************************************************/
#include "adc.h"
/*** AVR ***/
#include <avr/io.h>
#include <util/delay.h>


/***** FUNCTION PROTOTYPES ********************************************/
void adc_dummy_conversion(void);


/***** FUNCTIONS ******************************************************/
/***
 * Initialize the ADC.
 *
 * @param[in]   prescaler   The ADC clock prescaler option
 * @param[in]   reference   The ADC reference source option
 ***/
void adc_init(ADC_PRESCALER_t prescaler, ADC_AREF_t reference) {
    /* Initially, select input CH0 */
    adc_set_input(ADC_CH0);
    /* Set the prescaler */
    adc_set_prescaler(prescaler);
    /* Set the reference voltage */
    adc_set_reference(reference);
    /* Enable the ADC */
    adc_enable();
    /* Perform a dummy conversion */
    adc_dummy_conversion();
}


/***
 * De-initialize the ADC.
 ***/
void adc_deinit(void) {
    /* Clear the ADC's internal multiplexer */
    ADMUX = 0x00;
    /* Clear the control register */
    ADCSRA = 0x00;
}


/***
 * Enable the ADC peripheral.
 ***/
void adc_enable(void) {
    /* Enable the ADC */
    ADCSRA |= _BV(ADEN);
}


/***
 * Disable the ADC peripheral.
 ***/
void adc_disable(void) {
    /* Disable the ADC */
    ADCSRA &= ~_BV(ADEN);
}


/***
 * Select the ADC input.
 *
 * @param[in]   input       The input to be used
 ***/
void adc_set_input(ADC_INPUT_t input) {
    /* Select desired input with internal multiplexer */
    ADMUX = ((ADMUX & 0xE0) | (input & 0x0F));
}


/***
 * Select the ADC prescaler.
 *
 * @param[in]   prescaler   The prescaler to be used
 ***/
void adc_set_prescaler(ADC_PRESCALER_t prescaler) {
    /* Select desired prescaler */
    ADCSRA = ((ADCSRA & 0xF8) | (prescaler & 0x07));
}


/***
 * Select the ADC reference source.
 *
 * @param[in]   reference   The reference source to be used
 ***/
void adc_set_reference(ADC_AREF_t reference) {
    /* Select desired reference */
    ADMUX = ((ADMUX & 0x2F) | ((reference & 0x03) << 6));
}


/***
 * Read the currently selected input of the ADC.
 *
 * @return      10-bit conversion result (stored in 16-bit right aligned)
 ***/
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


/***
 * Read the given input of the ADC.
 *
 * @param[in]   input       The input to be read
 * @return      10-bit conversion result (stored in 16-bit right aligned)
 ***/
uint16_t adc_read_input(ADC_INPUT_t input) {
    /* Set the given input */
    adc_set_input(input);
    /* Return the conversion result */
    return adc_read();
}


/***
 * Perform a dummy conversion.
 ***/
void adc_dummy_conversion(void) {
    /* Perform a conversion but neglect the result */
    (void)adc_read();
}


/***
 * Measure the MCU's supply voltage internally.
 *
 * @return      Supply voltage in volts (V)
 ***/
float adc_read_vss(void) {
    /* Save the current ADMUX configuration */
    uint8_t reg = ADMUX;
    /* Set the register accordingly */
    ADMUX  = 0x4E;
    /* Give the reference some time to settle */
    _delay_ms(ADC_DELAY_CHANGE_REFERENCE);
    /* Perform a dummy conversion */
    adc_dummy_conversion();
    /* Store the converted ADC value */
    float result = (1.1 * (1023.0/(float)adc_read()));
    /* Restore the ADMUX configuration */
    ADMUX = reg;
    /* Return the result */
    return result;
}
