/**
 *  Header file for temperature helper functions.
 */

#ifndef _HELP_TEMP_H_
#define _HELP_TEMP_H_

/***** INCLUDES ***************************************************************/
#include <stdint.h>


/***** MACROS *****************************************************************/
/* 103JT thermistor (10k@25°C) */
#define JT103_BETA                      (3435)
#define JT103_R_ROOM                    (10000UL)
/* Resistor Network & ADC */
#define MAX_ADC                         (1023)
#define BALANCE_RESISTOR                (10000UL)
/* Temperature Conversion & Room Temperature */
#define TEMP_K2C                    (273.15)
#define TEMP_ROOM                   (TEMP_K2C + 25)

/* HI formula coefficients https://en.wikipedia.org/wiki/Heat_index#Formula */
#define TEMP_HI_C1                      (-8.78469475556)
#define TEMP_HI_C2                      (1.61139411)
#define TEMP_HI_C3                      (2.33854883889)
#define TEMP_HI_C4                      (-0.14611605)
#define TEMP_HI_C5                      (-0.012308094)
#define TEMP_HI_C6                      (-0.0164248277778)
#define TEMP_HI_C7                      (0.002211732)
#define TEMP_HI_C8                      (0.00072546)
#define TEMP_HI_C9                      (-0.000003582)


/***** GLOBAL VARIABLES *******************************************************/


/***** ENUMERATION ************************************************************/


/***** STRUCTURES *************************************************************/


/***** FUNCTION PROTOTYPES ****************************************************/
/* Thermistor */
float JT103_get_temperature(uint16_t adc_value);
/* Temperature conversion */
float temp_C2F(float temp);
float temp_F2C(float temp);
float temp_get_heatindex(float T, float R);


/***** INLINE FUNCTIONS *******************************************************/


#endif // _HELP_TEMP_H_
