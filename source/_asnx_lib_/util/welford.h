/*!
 * @brief   ASN(x) Welford's Algorithm library -- header file
 *
 * Library implementing Welford's Algorithm to calculate the
 * standard deviation of continuous data in an online manner.
 *
 * @file    /_asnx_lib_/util/welford.h
 * @author  Dominik Widhalm
 * @version 1.1.0
 * @date    2022/01/31
 */

#ifndef _ASNX_WELFORD_H_
#define _ASNX_WELFORD_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
#include <math.h>


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
void welford_init(welford_t* data);
float welford_get_mean(welford_t* data);
float welford_get_variance(welford_t* data);
float welford_get_stddev(welford_t* data);
void welford_add(welford_t* data, float value);
void welford_remove(welford_t* data, float value);
void welford_replace(welford_t* data, float value_o, float value_n);

#endif // _ASNX_WELFORD_H_
