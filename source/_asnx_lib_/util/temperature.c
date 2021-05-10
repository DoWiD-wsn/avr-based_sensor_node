/*****
 * @brief   ASN(x) temperature helper library
 *
 * Library to support working with temperature values and their conversion.
 *
 * @file    /_asnx_lib_/util/temperature.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1.0 $
 * @date    $Date: 2021/05/10 $
 *****/

/***** INCLUDES *******************************************************/
#include "temperature.h"


/***** FUNCTIONS ******************************************************/
/***
 * Convert a temperature in degree Celsius (°C) to degree Fahrenheit (°F).
 * 
 * @param[in]   temp    Temperature in degree Celsius (°C)
 * @return      Temperature in degree Fahrenheit (°F)
 ***/
float temp_C2F(float temp) {
    return (temp * 9.0 / 5.0 + 32.0);
}


/***
 * Convert a temperature in degree Fahrenheit (°F) to degree Celsius (°C).
 * 
 * @param[in]   temp    Temperature in degree Fahrenheit (°F)
 * @return      Temperature in degree Celsius (°C)
 ***/
float temp_F2C(float temp) {
    return ((temp - 32.0) * 5.0 / 9.0); 
}


/***
 * Compute the heatindex based on temperature and humidity.
 * 
 * @see http://en.wikipedia.org/wiki/Heat_index
 * @param[in]   T       Temperature in degree Celsius (°C)
 * @param[in]   R       Relative humidity in percent (%RH)
 * @return      Resulting heat index
 ***/
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
