/**
 *  Source file for temperature helper functions.
 */

/***** INCLUDES ***************************************************************/
/*** STD ***/
#include <math.h>
/*** OWN ***/
#include "temperature.h"


/***** GLOBAL VARIABLES *******************************************************/


/***** LOCAL FUNCTION PROTOTYPES **********************************************/


/***** WRAPPER FUNCTIONS ******************************************************/


/***** FUNCTIONS **************************************************************/
/*
 * Calculate the temperature depending on the thermistor resistance
 * 
 * uint16_t adc_value   ADC value
 * return               Temperature in degree Celsius
 */
float JT103_get_temperature(uint16_t adc_value) {
    /* Calculate the thermistor's resistance */
    float R_thermistor = (float)BALANCE_RESISTOR / (((float)MAX_ADC / (float)adc_value) - 1.0);
    /* Use the beta equation to get the temperature */
    return ((JT103_BETA * TEMP_ROOM) / (JT103_BETA + (TEMP_ROOM * log(R_thermistor/JT103_R_ROOM)))) - TEMP_K2C;
}

/*
 * Convert temperature in degree Celsius to degree Fahrenheit
 * 
 * float temp           Temperature in degree Celsius
 * return               Temperature in degree Fahrenheit
 */
float temp_C2F(float temp) {
    return (temp * 9.0 / 5.0 + 32.0);
}


/*
 * Convert temperature in degree Fahrenheit to degree Celsius
 * 
 * float temp           Temperature in degree Fahrenheit
 * return               Temperature in degree Celsius
 */
float temp_F2C(float temp) {
    return ((temp - 32.0) * 5.0 / 9.0); 
}


/*
 * Compute the heatindex based on temperature and humidity
 * 
 * See also: http://en.wikipedia.org/wiki/Heat_index
 * 
 * float T              Temperature in degree Celsius
 * float R              Relative humidity in percent
 * return               Heatindex
 */
float temp_get_heatindex(float T, float R) {
    return (TEMP_HI_C1) + 
           (TEMP_HI_C2 * T) + 
           (TEMP_HI_C3 * R) +
           (TEMP_HI_C4 * T*R) +
           (TEMP_HI_C5 * T*T) +
           (TEMP_HI_C6 * R*R) + 
           (TEMP_HI_C7 * T*T*R) + 
           (TEMP_HI_C8 * T*R*R) +
           (TEMP_HI_C9 * T*T*R*R);
}
