/*!
 * @brief   ASN(x) fixed-point arithmetic library -- header file
 *
 * Library to support the use of fixed point variables.
 *
 * @file    /_asnx_lib_/util/fixed_point.h
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 */

#ifndef _ASNX_FP_H_
#define _ASNX_FP_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>


/***** FUNCTION PROTOTYPES ********************************************/
/* Defines number of fractional bits */
float fp_fixed8_to_float(uint8_t input, uint8_t f_bits);
uint8_t fp_float_to_fixed8(float input, uint8_t f_bits);
float fp_fixed16_to_float(uint16_t input, uint8_t f_bits);
uint16_t fp_float_to_fixed16(float input, uint8_t f_bits);
double fp_fixed32_to_double(uint32_t input, uint8_t f_bits);
uint32_t fp_double_to_fixed32(double input, uint8_t f_bits);
/* Default number of fractional bits */
float fp_fixed8_to_float_2to6(uint16_t input);
uint8_t fp_float_to_fixed8_2to6(float input);
float fp_fixed16_to_float_10to6(uint16_t input);
uint16_t fp_float_to_fixed16_10to6(float input);
double fp_fixed32_to_double_26to6(uint32_t input);
uint32_t fp_double_to_fixed32_26to6(double input);

#endif // _ASNX_FP_H_
