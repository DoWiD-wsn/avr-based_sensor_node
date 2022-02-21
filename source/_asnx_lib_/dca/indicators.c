/*!
 * @brief   ASN(x) fault indicator library -- source file
 *
 * Implementation of the fault indicators available on the ASN(x).
 *
 * @file    /_asnx_lib_/dca/indicators.c
 * @author  Dominik Widhalm
 * @version 1.4.1
 * @date    2021/10/25
 */

/***** INCLUDES *******************************************************/
#include "indicators.h"


/***** GLOBAL VARIABLES ***********************************************/
/*! Battery voltage monitor value array */
float x_bat_values[X_BAT_N] = {0};
/*! Battery voltage monitor value array index (99 if array is not filled yet) */
uint8_t x_bat_index = 99;
/*! Active runtime monitor value array */
uint32_t x_art_values[X_ART_N] = {0};
/*! Active runtime monitor value array index (99 if array is not filled yet) */
uint8_t x_art_index = 99;
/*! Reset monitor previous value */
float x_rst_prev = 0.0;
/*! software incident counter value */
uint16_t x_ic = 0;


/***** FUNCTIONS ******************************************************/
/* General */
/*!
 * Initialize the fault indicators, that is, reset all values expect for
 * the reset monitor fault indicator. For this, read the value from the
 * EEPROM to have to last value before reset.
 */
void indicators_init(void) {
    /* Reset stored values */
    x_bat_reset();
    x_art_reset();
    x_rst_reset();
    x_ic_reset();
    /* Check if X_RST value in EEPROM was written before */
    if(eeprom_read_byte((const uint8_t *)X_RST_EEPROM) != 0xFF) {
        /* Read the previous reset-source indicator from EEPROM */
        x_rst_prev = eeprom_read_float((float *)X_RST_EEPROM);
        x_rst_set(x_rst_prev);
    }
}

/* Node temperature monitor (X_NT) */
/*!
 * Get the normalized node temperature monitor fault indicator (X_NT) value.
 * 
 * @param[in]   t_mcu       MCU surface temperature (°C)
 * @param[in]   t_brd       sensor node board temperature (°C)
 * @param[in]   t_trx       radio core temperature (°C)
 * @return      normalized temperature monitor fault indicator (X_NT) value
 */
float x_nt_get_normalized(float t_mcu, float t_brd, float t_trx) {
    /* Store the previous values */
    static float t_mcu_prev;
    static float t_brd_prev;
    static float t_trx_prev;
    /* Flag to indicator whether a previous measurement is available */
    static uint8_t have_prev = 0;
    /* Check if there were previous measurements */
    if(have_prev == 0) {
        /* Remember current values */
        t_mcu_prev = t_mcu;
        t_brd_prev = t_brd;
        t_trx_prev = t_trx;
        /* From now on, we have previous values */
        have_prev = 1;
        /* First run -> no difference to calculate */
        return 0.0;
    }
    /* Get the difference between the current and previous measurement */
    float D_mcu = t_mcu - t_mcu_prev;
    float D_brd = t_brd - t_brd_prev;
    float D_trx = t_trx - t_trx_prev;
    /* Calculate the mean value of the three temperature differences */
    float mu_nt = (D_mcu + D_brd + D_trx) / 3.0;
    /* Calculate the standard deviation of the three temperature differences */
    float sigma_nt = (float)sqrt(((double)(D_mcu-mu_nt)*(double)(D_mcu-mu_nt) + \
                                  (double)(D_brd-mu_nt)*(double)(D_brd-mu_nt) + \
                                  (double)(D_trx-mu_nt)*(double)(D_trx-mu_nt) \
                                 ) / 3.0);
    /* Remember previous measurements */
    t_mcu_prev = t_mcu;
    t_brd_prev = t_brd;
    t_trx_prev = t_trx;
    /* Return X_NT with normalized standard deviation (max: 1.0) */
    return fmin((sigma_nt/X_NT_MAX), 1.0);
}


/* Supply voltage monitor (X_VS) */
/*!
 * Get the normalized supply voltage monitor fault indicator (X_VS) value.
 * 
 * @param[in]   v_mcu       MCU supply voltage level (V)
 * @param[in]   v_trx       radio supply voltage level (V)
 * @return      normalized supply voltage monitor fault indicator (X_VS) value
 */
float x_vs_get_normalized(float v_mcu, float v_trx) {
    /* Calculate and return the normalized difference between both voltages (max: 1.0) */
    if(v_mcu > v_trx) {
        return fmin(((v_mcu-v_trx) / X_VS_MAX), 1.0);
    } else {
        return fmin(((v_trx-v_mcu) / X_VS_MAX), 1.0);
    }
}


/* Battery voltage monitor (X_BAT) */
/*!
 * Reset the battery voltage monitor fault indicator (X_BAT) value.
 */
void x_bat_reset(void) {
    /* Clear array of previous values */
    for(uint8_t i=0; i<X_BAT_N; i++) {
        x_bat_values[i] = 0.0;
    }
    /* Set array index to "not used yet" */
    x_bat_index = 99;
}

/*!
 * Get the normalized battery voltage monitor fault indicator (X_BAT) value.
 * 
 * @param[in]   v_bat       battery voltage level (V)
 * @return      normalized battery voltage monitor fault indicator (X_BAT) value
 */
float x_bat_get_normalized(float v_bat) {
    /* Check if value array is empty */
    if(x_bat_index==99) {
        /* Fill entire array with current value */
        for(uint8_t i=0; i<X_BAT_N; i++) {
            x_bat_values[i] = v_bat;
        }
        /* Set array index to 1 */
        x_bat_index = 1;
    } else {
        /* Update oldest value with new one */
        x_bat_values[x_bat_index] = v_bat;
        /* Update array index */
        x_bat_index = (x_bat_index+1) % X_BAT_N;
    }
    /* Calculate mean value */
    double x_bat_mean = 0;
    for(uint8_t i=0; i<X_BAT_N; i++) {
        x_bat_mean += (double)x_bat_values[i];
    }
    x_bat_mean /= X_BAT_N;
    /* Calculate standard deviation */
    double x_bat_stddev = 0;
    for(uint8_t i=0; i<X_BAT_N; i++) {
        x_bat_stddev += (((double)x_bat_values[i]-x_bat_mean) * ((double)x_bat_values[i]-x_bat_mean));
    }
    x_bat_stddev /= X_BAT_N;
    x_bat_stddev = sqrt(x_bat_stddev);
    /* Return normalized X_BAT (max: 1.0) */
    return fmin((float)(x_bat_stddev/X_BAT_MAX), 1.0);
}


/* Active runtime monitor (X_ART) */
/*!
 * Reset the active runtime monitor fault indicator (X_ART) value.
 */
void x_art_reset(void) {
    /* Clear array of previous values */
    for(uint8_t i=0; i<X_ART_N; i++) {
        x_art_values[i] = 0;
    }
    /* Set array index to "not used yet" */
    x_art_index = 99;
}

/*!
 * Get the normalized active runtime monitor fault indicator (X_ART) value.
 * 
 * @param[in]   t_art       length of the last active phase (ms)
 * @return      normalized active runtime monitor fault indicator (X_ART) value
 */
float x_art_get_normalized(uint16_t t_art) {
    /* Check if value array is empty */
    if(x_art_index==99) {
        /* Fill entire array with current value */
        for(uint8_t i=0; i<X_ART_N; i++) {
            x_art_values[i] = t_art;
        }
        /* Set array index to 1 */
        x_art_index = 1;
    } else {
        /* Update oldest value with new one */
        x_art_values[x_art_index] = t_art;
        /* Update array index */
        x_art_index = (x_art_index+1) % X_ART_N;
    }
    /* Calculate mean value */
    double x_art_mean = 0;
    for(uint8_t i=0; i<X_ART_N; i++) {
        x_art_mean += (double)x_art_values[i];
    }
    x_art_mean /= X_ART_N;
    /* Calculate standard deviation */
    double x_art_stddev = 0;
    for(uint8_t i=0; i<X_ART_N; i++) {
        x_art_stddev += (((double)x_art_values[i]-x_art_mean) * ((double)x_art_values[i]-x_art_mean));
    }
    x_art_stddev /= X_ART_N;
    x_art_stddev = sqrt(x_art_stddev);
    /* Get magnitude of std-dev */
    float x_art_mag = 0.0;
    if(x_art_stddev>1.0) {
        x_art_mag = log10(x_art_stddev);
    }
    /* Return normalized X_ART depending on magnitude of difference (max: 1.0) */
    return fmin((float)(x_art_mag/X_ART_MAX), 1.0);
}


/* Reset monitor (X_RST) */
/*!
 * Reset the reset monitor fault indicator (X_RST) value.
 */
void x_rst_reset(void) {
    x_rst_prev = 0;
}

/*!
 * Set the reset monitor fault indicator (X_RST) value.
 * 
 * @param[in]   x_rst       new value of X_RST
 */
void x_rst_set(float x_rst) {
     x_rst_prev = x_rst;
}

/*!
 * Get the normalized reset monitor fault indicator (X_RST) value.
 * 
 * @param[in]   mcusr       value of the AVR's MCUSR
 * @return      normalized reset monitor fault indicator (X_RST) value
 */
float x_rst_get_normalized(uint8_t mcusr) {
    /* Decrease previous value of X_RST */
    float x_rst = (x_rst_prev * X_RST_DECAY);
    /* Add normalized MCUSR value */
    x_rst += ((float)(mcusr & 0x0F) / X_RST_MAX);
    /* Update reset monitor fault indicator (X_RST) value in EEPROM if value changed notably */
    float a = (x_rst > x_rst_prev) ? x_rst : x_rst_prev;
    float b = (x_rst > x_rst_prev) ? x_rst_prev : x_rst;
    if((a-b) > X_RST_UPDATE) {
        eeprom_write_float((float *)X_RST_EEPROM,x_rst);
    }
    /* Check the final value (max: 1.0) */
    x_rst = fmin(x_rst, 1.0);
    /* Updated stored previous value */
    x_rst_prev = x_rst;
    /* Return updated value */
    return x_rst;
}


/* Software incident counter (X_IC) */
/*!
 * Reset the software incident counter fault indicator (X_IC) value.
 */
void x_ic_reset(void) {
    x_ic = 0;
}

/*!
 * Increment the software incident counter fault indicator (X_IC) value.
 *
 * @param[in]   value   Value by which X_IC should be incremented
 */
void x_ic_inc(uint8_t value) {
    x_ic += value;
}

/*!
 * Decrement the software incident counter fault indicator (X_IC) value.
 *
 * @param[in]   value   Value by which X_IC should be decremented
 */
void x_ic_dec(uint8_t value) {
    /* Check if X_IC is bigger than the given value */
    if(x_ic > value) {
        x_ic -= value;
    } else {
        x_ic = 0;
    }
}

/*!
 * Get the software incident counter fault indicator (X_IC) value.
 * 
 * @return      software incident counter fault indicator (X_IC) value
 */
uint16_t x_ic_get(void) {
    /* Return the value */
    return x_ic;
}

/*!
 * Get the normalized software incident counter fault indicator (X_IC) value.
 * 
 * @return      normalized software incident counter fault indicator (X_IC) value
 */
float x_ic_get_normalized(void) {
    /* Return normalized value (max: 1.0) */
    return fmin(((float)x_ic / X_IC_MAX), 1.0);
}


/* ADC self-check (X_ADC) */
/*!
 * Get the normalized ADC self-check fault indicator (X_ADC) value.
 * 
 * @param[in]   adc_value   latest ADC conversion result
 * @return      normalized ADC self-check fault indicator (X_ADC) value
 */
float x_adc_get_normalized(uint16_t adc_value) {
    /* Calculate and return the normalized difference between acquired and expected value (max: 1.0) */
    if(adc_value > X_ADC_EXPECTED) {
        return fmin(((adc_value-X_ADC_EXPECTED) / X_ADC_MAX), 1.0);
    } else {
        return fmin(((X_ADC_EXPECTED-adc_value) / X_ADC_MAX), 1.0);
    }
}


/* USART self-check (X_USART) */
/*!
 * Get the normalized USART self-check fault indicator (X_USART) value.
 * 
 * @param[in]   tx          pointer to the transmit buffer array
 * @param[in]   rx          pointer to the receive buffer array
 * @param[in]   len         length of both arrays (should be identical)
 * @return      normalized USART self-check fault indicator (X_USART) value
 */
float x_usart_get_normalized(uint8_t* tx, uint8_t* rx, uint8_t len) {
    /* Variable to count number of different bytes */
    uint8_t diff = 0;
    /* Iterate over the arrays */
    for(uint8_t i=0; i<len; i++) {
        /* Check if received byte matches transmit value */
        if(tx[i] != rx[i]) {
            diff++;
        }
    }
    /* Normalize and return value (max: 1.0) */
    return fmin(((float)diff / X_USART_MAX), 1.0);
}
