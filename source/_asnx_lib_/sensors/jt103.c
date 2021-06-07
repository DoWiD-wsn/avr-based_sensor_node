/*!
 * @brief   ASN(x) 103JT thermistor library -- source file
 *
 * Library to support the use of the 103JT thermistor for temperature measurements.
 *
 * @file    /_asnx_lib_/sensors/jt103.c
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 */

/***** INCLUDES *******************************************************/
#include "jt103.h"


/***** FUNCTIONS ******************************************************/
/*!
 * Calculate the temperature depending on the thermistor resistance using the Steinhart-Hart equation.
 * 
 * @param[in]   adc_value   ADC conversion result
 * @return      Temperature in degree Celsius (°C)
 *
 * @see         https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
 */
float jt103_get_temperature(uint16_t adc_value) {
    /* Calculate the thermistor's resistance */
    float R_thermistor = (float)JT103_R_BALANCE / (((float)JT103_ADC_MAX / (float)adc_value) - 1.0);
    /* Use the beta equation to get the temperature */
    return ((JT103_BETA * JT103_T_ROOM) / (JT103_BETA + (JT103_T_ROOM * log(R_thermistor/JT103_R_ROOM)))) - JT103_T_K2C;
}
