/*!
 * @brief   ASN(x) Welford's Algorithm library -- header file
 *
 * Library implementing Welford's Algorithm to calculate the
 * standard deviation of continuous data in an online manner.
 *
 * @file    /_asnx_lib_/util/welford.h
 * @author  Dominik Widhalm
 * @version 1.0.1
 * @date    2021/12/27
 * 
 * @see https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
 * @see https://www.kite.com/python/answers/how-to-find-a-running-standard-deviation-in-python
 * @see https://gist.github.com/qubyte/4064710
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
