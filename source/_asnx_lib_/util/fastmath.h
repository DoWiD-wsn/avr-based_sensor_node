/*!
 * @brief   ASN(x) fast math library -- header file
 *
 * Library with fast alternatives and/or approximations for
 * computational-intense mathematical functions.
 *
 * @file    /_asnx_lib_/util/fastmath.h
 * @author  Dominik Widhalm
 * @version 1.0.0
 * @date    2021/12/27
 */

#ifndef _ASNX_FASTMATH_H_
#define _ASNX_FASTMATH_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>


/***** MACROS *********************************************************/
/* SQRT */
/*! Default number of iterations for Newton's Method */
#define SQRT_APPROX_ITERATIONS      5
/* STD-DEV */
/*! Use the SQRT hack (1) or Newton's Method (0) */
#define STDDEV_USE_SQRT_HACK        0


/***** STRUCTURES *****************************************************/
/*!
 * A structure to hold the data for Welford's Algorithm.
 */
typedef struct {
    float mean;                 /**< Cumulative mean */
    float work;                 /**< Cumulative working data */
    uint8_t cnt;                /**< Number of values included */
} welford_t;


/***** FUNCTION PROTOTYPES ********************************************/
/* LOG10 */
float log10_approx(float value);
/* SQRT */
float sqrt_hack(float value);
float sqrt_approx(float value, uint8_t iterations);
/* STD-DEV */
void welford_init(welford_t* data);
float welford_get_variance(welford_t* data);
float welford_get_stddev(welford_t* data);
void welford_add(welford_t* data, float value);
void welford_remove(welford_t* data, float value);
void welford_replace(welford_t* data, float value_o, float value_n);

#endif // _ASNX_FASTMATH_H_
