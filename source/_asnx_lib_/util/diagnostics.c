/*!
 * @brief   ASN(x) self-diagnostics library -- source file
 *
 * Library to support the self-diagnostic measures of the ASN(x).
 *
 * @file    /_asnx_lib_/util/diagnostics.c
 * @author  Dominik Widhalm
 * @version 1.4.0
 * @date    2021/12/29
 */

/***** INCLUDES *******************************************************/
#include "diagnostics.h"


/***** GLOBAL VARIABLES ***********************************************/
/*! GPIO struct for the R_divider enable signal */
hw_io_t rden = {NULL, NULL, NULL, 0};
/*! Reset-source value */
float rsource = 0.0;


/***** FUNCTIONS ******************************************************/
/*!
 * Initialize the diagnostic measures (i.e., the enable-MOSFET signal).
 * Initially, the diagnostic measures are deactivated (enable signal set to low).
 */
void diag_init(void) {
    /* Fill the R_divider enable signal structure */
    hw_get_io(&rden, &DIAG_RDEN_DDR, &DIAG_RDEN_PORT, &DIAG_RDEN_PIN, DIAG_RDEN_GPIO);
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
    return adc_read_input(DIAG_ADC_CH);
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
 * @param[in]   vcc     VCC voltage level (V)
 * @return      Battery voltage in volts (V)
 */
float diag_read_vbat(float vcc) {
    return 2.0 * (adc_read_input(DIAG_VBAT_CH) * (vcc / 1023.0));
}


/*!
 * Read the MCU surface temperature using the thermistor via ADC.
 *
 * @return      MCU surface temperature in degrees Celsius (°C)
 */
float diag_read_tsurface(void) {
    return jt103_get_temperature(adc_read_input(DIAG_TMCU_CH));
}


/*!
 * Reset the reset-source indicator value.
 * Uses the global "rsource" variable.
 */
void diag_rsource_reset(void) {
    /* Reset value */
    rsource = 0.0;
}


/*!
 * Set the reset-source indicator to a given value.
 * Uses the global "rsource" variable.
 *
 * @param[in]   value       Desired indicator value
 */
void diag_rsource_set(float value) {
    /* Set value */
    rsource = value;
}


/*!
 * Update the reset-source indicator based on the current MCUSR value.
 * Uses the global "rsource" variable.
 *
 * @param[in]   mcusr       Current MCUSR value
 * @return      Updated reset-source indicator
 */
float diag_rsource_update(uint8_t mcusr) {
    /* Decrease previous value */
    rsource = (rsource * DIAG_DECAY_RATE);
    /* Add value based on MCUSR */
    if(mcusr & _BV(WDRF)) {
        /* Watchdog */
        rsource += DIAG_DECAY_WDRF;
    } else if(mcusr & _BV(BORF)) {
        /* Brown-out */
        rsource += DIAG_DECAY_BORF;
    } else if(mcusr & _BV(EXTRF)) {
        /* External reset */
        rsource += DIAG_DECAY_EXTRF;
    } else if(mcusr & _BV(PORF)) {
        /* Power-on */
        rsource += DIAG_DECAY_PORF;
    }
    /* Check thresholds */
    if(rsource < DIAG_DECAY_MIN) {
        rsource = 0.0;
    } else if(rsource > DIAG_DECAY_MAX) {
        rsource = DIAG_DECAY_MAX;
    }
    /* Return updated value */
    return rsource;
}
