/*!
 * @brief   ASN(x) 103JT thermistor library -- header file
 *
 * Library to support the use of the 103JT thermistor for temperature measurements.
 *
 * @file    /_asnx_lib_/sensors/jt103.h
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 */

#ifndef _ASNX_JT103_H_
#define _ASNX_JT103_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <math.h>
#include <stdint.h>


/***** DEFINES ********************************************************/
/* 103JT thermistor (10k@25°C) */
#define JT103_BETA              (3435)
#define JT103_R_ROOM            (10000UL)
/* Resistor Network & ADC */
#define JT103_R_BALANCE         (10000UL)
#define JT103_ADC_MAX           (1023)
/* Temperature Conversion & Room Temperature */
#define JT103_T_K2C             (273.15)
#define JT103_T_ROOM            (JT103_T_K2C + 25)


/***** FUNCTION PROTOTYPES ********************************************/
float jt103_get_temperature(uint16_t adc_value);


#endif // _ASNX_JT103_H_
