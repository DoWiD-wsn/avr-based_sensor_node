/*!
 * @brief   ASN(x) temperature helper library -- header file
 *
 * Library to support working with temperature values and their conversion.
 *
 * @file    /_asnx_lib_/util/temperature.h
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 */

#ifndef _ASNX_TEMP_H_
#define _ASNX_TEMP_H_

/***** DEFINES ********************************************************/
/* Heat-index (HI) formula coefficients
 * @see https://en.wikipedia.org/wiki/Heat_index#Formula
 */
#define TEMP_HI_C1                      (-8.78469475556)
#define TEMP_HI_C2                      (1.61139411)
#define TEMP_HI_C3                      (2.33854883889)
#define TEMP_HI_C4                      (-0.14611605)
#define TEMP_HI_C5                      (-0.012308094)
#define TEMP_HI_C6                      (-0.0164248277778)
#define TEMP_HI_C7                      (0.002211732)
#define TEMP_HI_C8                      (0.00072546)
#define TEMP_HI_C9                      (-0.000003582)


/***** FUNCTION PROTOTYPES ********************************************/
/* Temperature conversion */
float temp_C2F(float temp);
float temp_F2C(float temp);
/* Calculate the heat index (°C) */
float temp_get_heatindex(float T, float R);


#endif // _ASNX_TEMP_H_
