/*!
 * @brief   ASN(x) self-diagnostics library -- source file
 *
 * Library to support the self-diagnostic measures of the ASN(x).
 *
 * @file    /_asnx_lib_/util/diagnostics.c
 * @author  Dominik Widhalm
 * @version 1.3.0
 * @date    2021/08/09
 */

/***** INCLUDES *******************************************************/
#include "diagnostics.h"


/***** GLOBAL VARIABLES ***********************************************/
/*! GPIO struct for the R_divider enable signal */
hw_io_t rden = {NULL, NULL, NULL, 0};


/***** FUNCTIONS ******************************************************/
/*!
 * Initialize the diagnostic measures (i.e., the enable-MOSFET signal).
 * Initially, the diagnostic measures are deactivated (enable signal set to low).
 */
void diag_init(void) {
    /* Fill the R_divider enable signal structure */
    hw_get_io(&rden, &RDEN_DDR, &RDEN_PORT, &RDEN_PIN, RDEN_GPIO);
    /* Set the R_divider enable signal to output */
    hw_set_output(&rden);
    /* Set the R_divider enable signal state to low */
    hw_set_output_low(&rden);
}


/*!
 * Enable the diagnostic measures (enable signal set to high).
 */
void diag_enable(void) {
    /* Set the R_divider enable signal state to high */
    hw_set_output_high(&rden);
}


/*!
 * Disable the diagnostic measures (enable signal set to low).
 */
void diag_disable(void) {
    /* Set the R_divider enable signal state to low */
    hw_set_output_low(&rden);
}


/*!
 * Read the self-test value of the ADC.
 *
 * @return      10-bit conversion result (stored in 16-bit right aligned)
 */
uint16_t diag_adc_check(void){
    /* Basically, return the ADC value of channel 0 */
    return adc_read_input(ADC_CH0);
}


/*!
 * Read the MCU's supply voltage internally.
 *
 * @return      Supply voltage in volts (V)
 */
float diag_read_vcc(void) {
    /* Use the function provided by the ADC module */
    return adc_read_vcc();
}


/*!
 * Read the battery voltage via ADC.
 *
 * @return      Battery voltage in volts (V)
 */
float diag_read_vbat(void) {
    return 2.0 * (adc_read_input(ADC_CH1) * (diag_read_vcc() / 1023.0));
}