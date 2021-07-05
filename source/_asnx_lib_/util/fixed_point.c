/*!
 * @brief   ASN(x) fixed-point arithmetic library -- source file
 *
 * Library to support the use of fixed point variables.
 *
 * @file    /_asnx_lib_/util/fixed_point.c
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 */

/***** INCLUDES *******************************************************/
#include "fixed_point.h"


/***** FUNCTIONS ******************************************************/
/*!
 * Convert a 16-bit fixed-point number to floating-point.
 * 
 * @param[in]   input       16-bit fixed-point number
 * @param[in]   f_bits      Number of fractional bits
 * @return      Corresponding floating-point number
 */
float fp_fixed16_to_float(uint16_t input, uint8_t f_bits) {
    /* Convert fixed16 to float */
    float tmp = ((float)(input & 0x7FFF) / (float)(1 << f_bits));
    /* Check sign of input */
    if(input & 0x8000) {
        tmp *= -1;
    }
    /* Return the float value */
    return tmp;
}


/*!
 * Convert a floating-point number to 16-bit fixed-point.
 * 
 * @param[in]   input       Floating-point number
 * @param[in]   f_bits      Number of fractional bits
 * @return      Corresponding 16-bit fixed-point number
 */
uint16_t fp_float_to_fixed16(float input, uint8_t f_bits) {
    uint16_t tmp = 0x0000;
    /* Check sign of input */
    if(input<0) {
        input *= -1;
        tmp |= 0x8000;
    }
    /* Convert float to fixed16 */
    tmp |= (uint16_t)(input * (1 << f_bits)) & 0x7FFF;
    /* Return the fixed16 value */
    return tmp;
}


/*!
 * Convert a 32-bit fixed-point number to floating-point.
 * 
 * @param[in]   input       32-bit fixed-point number
 * @param[in]   f_bits      Number of fractional bits
 * @return      Corresponding floating-point number
 */
double fp_fixed32_to_double(uint32_t input, uint8_t f_bits) {
    /* Convert fixed32 to double */
    double tmp = ((double)(input & 0x7FFFFFFF) / (double)(1 << f_bits));
    /* Check sign of input */
    if(input & 0x80000000) {
        tmp *= -1;
    }
    /* Return the double value */
    return tmp;
}


/*!
 * Convert a floating-point number to 32-bit fixed-point.
 * 
 * @param[in]   input       Floating-point number
 * @param[in]   f_bits      Number of fractional bits
 * @return      Corresponding 32-bit fixed-point number
 */
uint32_t fp_double_to_fixed32(double input, uint8_t f_bits) {
    uint32_t tmp = 0x00000000;
    /* Check sign of input */
    if(input<0) {
        input *= -1;
        tmp |= 0x80000000;
    }
    /* Convert float to fixed32 */
    tmp |= (uint32_t)(input * (1 << f_bits)) & 0x7FFFFFFF;
    /* Return the fixed32 value */
    return tmp;
}


/*!
 * Convert a 16-bit fixed-point number to floating-point with the default number fractional bits.
 * 
 * @param[in]   input       16-bit fixed-point number
 * @return      Corresponding floating-point number
 */
inline float fp_fixed16_to_float_10to6(uint16_t input) {
    return fp_fixed16_to_float(input, FP_FRACTIONAL_BITS_DEFAULT);
}


/*!
 * Convert a floating-point number to 16-bit fixed-point with the default number fractional bits.
 * 
 * @param[in]   input       Floating-point number
 * @return      Corresponding 16-bit fixed-point number
 */
inline uint16_t fp_float_to_fixed16_10to6(float input) {
    return fp_float_to_fixed16(input, FP_FRACTIONAL_BITS_DEFAULT);
}


/*!
 * Convert a 32-bit fixed-point number to floating-point with the default number fractional bits.
 * 
 * @param[in]   input       32-bit fixed-point number
 * @return      Corresponding floating-point number
 */
inline double fp_fixed32_to_double_26to6(uint32_t input) {
    return fp_fixed32_to_double(input, FP_FRACTIONAL_BITS_DEFAULT);
}


/*!
 * Convert a floating-point number to 32-bit fixed-point with the default number fractional bits.
 * 
 * @param[in]   input       Floating-point number
 * @return      Corresponding 32-bit fixed-point number
 */
inline uint32_t fp_double_to_fixed32_26to6(double input) {
    return fp_double_to_fixed32(input, FP_FRACTIONAL_BITS_DEFAULT);
}
