/*!
 * @brief   ASN(x) Welford's Algorithm library -- source file
 *
 * Library implementing Welford's Algorithm to calculate the
 * standard deviation of continuous data in an online manner.
 *
 * @file    /_asnx_lib_/util/welford.c
 * @author  Dominik Widhalm
 * @version 1.0.1
 * @date    2021/12/27
 * 
 * @see https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
 * @see https://www.kite.com/python/answers/how-to-find-a-running-standard-deviation-in-python
 * @see https://gist.github.com/qubyte/4064710
 */

/***** INCLUDES *******************************************************/
#include "welford.h"


/***** FUNCTIONS ******************************************************/
/*!
 * Initialize data structure for Welford's Algorithm.
 * 
 * @param[in]   data        Pointer to the Welford data structure
 */
void welford_init(welford_t* data) {
    data->mean = 0.0;
    data->work = 0.0;
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
    /* Return the standard deviation */
    return sqrt(data->work / data->cnt);
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
    data->mean += (value - data->mean) / data->cnt;
    /* Update working data */
    data->work += (value - data->mean) * (value - mean_old);
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
    /* Update working data */
    data->work -= (value - data->mean) * (value - mean_old);
    /* Update mean value */
    data->mean = mean_old;
    /* Decrement number of values included */
    data->cnt -= 1;
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
    /* Check if there is only one value */
    if(data->cnt == 1) {
        /* New mean is the new value */
        data->mean = value_n;
        /* Working data is zero */
        data->work = 0.0;
        /* Done */
        return;
    }
    /* Remove old value */
    welford_remove(data,value_o);
    /* Add new value */
    welford_add(data,value_n);
    
    // TODO: the following code was found, but does not work!?
    
    ///* Intermediate mean differences */
    //float delta_no = value_n - value_o;
    //float delta_o = value_o - data->mean;
    //float delta_n = value_n - data->mean;
    ///* Update mean value */
    //data->mean = data->mean + delta_no / data->cnt;
    ///* Get mean value without new value */
    //float delta_np = value_n - data->mean;
    ///* Update working data */
    //data->work = data->work - data->cnt / (data->cnt-1) * (delta_o * delta_o - delta_n * delta_np) - delta_no * delta_np / (data->cnt-1);
}

