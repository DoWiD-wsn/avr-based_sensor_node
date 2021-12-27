/*!
 * @brief   ASN(x) fast math library -- source file
 *
 * Library with fast alternatives and/or approximations for
 * computational-intense mathematical functions.
 *
 * @file    /_asnx_lib_/util/fastmath.c
 * @author  Dominik Widhalm
 * @version 1.0.0
 * @date    2021/12/27
 */

/***** INCLUDES *******************************************************/
#include "fastmath.h"


/***** FUNCTIONS ******************************************************/
/*!
 * Fast approximation of log10 using Taylor-series (for positive numbers).
 * 
 * Let:
 *   x = m × 10^p.
 * where 1 ≤ m ≤ 10. Then
 *   log10(x) = log10(m) + p
 * and so without loss of generality we can assume 1 ≤ m ≤ 10.
 * For m in this range we approximate log10(m) with:
 *   log10(m) ≈ (m - 1)/(m + 1)
 * ATTENTION: originally defined for the interval [1/sqrt(10), sqrt(10)]
 * This approximation has an average error of 5%.
 * 
 * @see         https://www.johndcook.com/blog/2021/03/22/mentally-calculating-logs/
 * @see         https://www.johndcook.com/blog/2021/03/24/log10-trick/
 * @see         http://www.phy6.org/stargaze/Slog4.htm
 * 
 * @param[in]   value       Input value (number)
 * @return      Approximated log10 of the given number
 */
float log10_approx(float value) {
    int exp = 0;
    /* Check if zero or less is given */
    if(value <= 0.0) {
        return 0;
    }
    /* Get decimal power */
    while(value >= 10) {
        value /= 10;
        exp += 1;
    }
    /* Approximate log(m) */
    value = (value-1) / (value+1);
    /* Return resulting value */
    return ((float)exp + value);
}


/*!
 * Fast hack to get sqrt for IEEE-754-based floats (average error 3%).
 * 
 * @see         https://stackoverflow.com/questions/43120045/how-does-this-float-square-root-approximation-work
 * 
 * @param[in]   value       Input value (number)
 * @return      Square root of the given number
 */
float sqrt_hack(float value) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
    const uint32_t result = 0x1fbb4000 + (*(uint32_t*)&value >> 1);
    return *(float*)&result;
#pragma GCC diagnostic pop
}


/*!
 * Fast approximation of sqrt using Newton's Method.
 * 
 * @see         https://www.goeduhub.com/3398/python-program-to-find-the-square-root-number-newtons-method
 * 
 * @param[in]   value       Input value (number)
 * @param[in]   iterations  Number of iterations
 * @return      Approximated square root of the given number
 */
float sqrt_approx(float value, uint8_t iterations) {
    float temp = value;
    /* Approximate value iteratively */
    for(uint8_t i=0; i<iterations; i++) {
        temp = 0.5 * (temp + value / temp);
    }
    /* Return result */
    return temp;
}


/**********************************************************************
 * Welford's Algorithm to calculate the standard deviation in an online manner.
 * @see https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
 * @see https://www.kite.com/python/answers/how-to-find-a-running-standard-deviation-in-python
 * @see https://gist.github.com/qubyte/4064710
 **********************************************************************/

/*!
 * Initialize data structure for Welford's Algorithm.
 * 
 * @param[in]   data        Pointer to the Welford data structure
 */
void welford_init(welford_t* data) {
    data->mean = 0.0;
    data->work  = 0.0;
    data->cnt  = 0;
}


/*!
 * Return the variance using Welford's Algorithm.
 * 
 * @param[in]   data        Pointer to the Welford data structure
 * @return      Variance of the given data structure.
 */
float welford_get_variance(welford_t* data) {
    /* Check if data structure is "empty" */
    if(data->cnt == 0) {
        return 0.0;
    }
    return (data->work / data->cnt);
}


/*!
 * Return the standard-deviation using Welford's Algorithm.
 * 
 * @param[in]   data        Pointer to the Welford data structure
 * @return      Standard deviation of the given data structure.
 */
float welford_get_stddev(welford_t* data) {
    /* Check if data structure is "empty" */
    if(data->cnt == 0) {
        return 0.0;
    }
#if STDDEV_USE_SQRT_HACK
    return sqrt_hack(data->work / data->cnt);
#else
    return sqrt_approx((data->work / data->cnt), SQRT_APPROX_ITERATIONS);
#endif
}


/*!
 * Add a value to the Welford data structure.
 * 
 * @param[in]   data        Pointer to the Welford data structure
 * @param[in]   value       Value to be added to the data structure
 */
void welford_add(welford_t* data, float value) {
    /* Store previous mean */
    float mean_old = data->mean;
    /* Increment number of values included */
    data->cnt += 1;
    /* Update mean value */
    data->mean = data->mean + (value - data->mean) / data->cnt;
    /* Update working data */
    data->work = data->work + (value - data->mean) * (value - mean_old);
}


/*!
 * Remove a value to the Welford data structure.
 * 
 * @param[in]   data        Pointer to the Welford data structure
 * @param[in]   value       Value to be removed from the data structure
 */
void welford_remove(welford_t* data, float value) {
    /* Check if data structure is "empty" */
    if(data->cnt == 0) {
        return;
    }
    /* Check if data structure will be "empty" */
    if(data->cnt == 1) {
        /* Clear data structure (re-init) */
        welford_init(data);
        /* Done */
        return;
    }
    /* Calculate previous mean */
    float mean_old = (data->cnt * data->mean - value) / (data->cnt - 1);
    /* Decrement number of values included */
    data->cnt -= 1;
    /* Update working data */
    data->work = (value - data->mean) * (value - mean_old);
    /* Update mean value */
    data->mean = mean_old;
}


/*!
 * Replace a value in the Welford data structure.
 * 
 * @param[in]   data        Pointer to the Welford data structure
 * @param[in]   value_o     Value to be removed from the data structure
 * @param[in]   value_n     Value to be added to the data structure
 */
void welford_replace(welford_t* data, float value_o, float value_n) {
    /* Check if data structure is "empty" */
    if(data->cnt == 0) {
        return;
    }
    /* Intermediate mean differences */
    float delta_no = value_n - value_o;
    float delta_o = value_o - data->mean;
    float delta_n = value_n - data->mean;
    /* Update mean value */
    data->mean = data->mean + delta_no / data->cnt;
    /* Get mean value without new value */
    float delta_np = value_n - data->mean;
    /* Update working data */
    data->work = data->work - data->cnt / (data->cnt-1) * (delta_o * delta_o - delta_n * delta_np) - delta_no * delta_np / (data->cnt-1);
}

