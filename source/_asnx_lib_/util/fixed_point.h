/*****
 * @brief   ASN(x) fixed-point arithmetic library
 *
 * Library to support the use of fixed point variables.
 *
 * @file    /_asnx_lib_/util/fixed_point.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1.0 $
 * @date    $Date: 2021/05/10 $
 *****/

#ifndef _ASNX_FP_H_
#define _ASNX_FP_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>


/***** DEFINES ********************************************************/
/* Default number of fractional bits (10.6 / 26.6 format) */
#define FP_FRACTIONAL_BITS_DEFAULT      6


/***** FUNCTION PROTOTYPES ********************************************/
/* Defines number of fractional bits */
float fp_fixed16_to_float(uint16_t input, uint8_t f_bits);
uint16_t fp_float_to_fixed16(float input, uint8_t f_bits);
double fp_fixed32_to_double(uint32_t input, uint8_t f_bits);
uint32_t fp_double_to_fixed32(double input, uint8_t f_bits);
/* Default number of fractional bits */
float fp_fixed16_to_float_10to6(uint16_t input);
uint16_t fp_float_to_fixed16_10to6(float input);
double fp_fixed32_to_double_26to6(uint32_t input);
uint32_t fp_double_to_fixed32_26to6(double input);

#endif // _ASNX_FP_H_
