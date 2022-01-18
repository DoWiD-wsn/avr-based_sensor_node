/*!
 * @brief   ASN(x) fault indicator library -- source file
 *
 * Implementation of the fault indicators available on the ASN(x).
 *
 * @file    /_asnx_lib_/dca/indicators.c
 * @author  Dominik Widhalm
 * @version 1.4.4
 * @date    2022/01/18
 */

/***** INCLUDES *******************************************************/
#include "indicators.h"


/***** GLOBAL VARIABLES ***********************************************/
/*! Battery voltage monitor value array */
float x_bat_values[X_BAT_N] = {0};
/*! Battery voltage monitor value array index */
uint8_t x_bat_index = 0;
/*! Battery voltage monitor Welford structure */
welford_t x_bat_data;
/*! Active runtime monitor value array */
uint32_t x_art_values[X_ART_N] = {0};
/*! Active runtime monitor value array index */
uint8_t x_art_index = 0;
/*! Active runtime monitor Welford structure */
welford_t x_art_data;
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
    return fmin(1.0, (sigma_nt/X_NT_MAX));
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
        return fmin(1.0, ((v_mcu-v_trx) / X_VS_MAX));
    } else {
        return fmin(1.0, ((v_trx-v_mcu) / X_VS_MAX));
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
    /* Set array index to zero */
    x_bat_index = 0;
    /* Initialize Welford data structure */
    welford_init(&x_bat_data);
}

/*!
 * Get the normalized battery voltage monitor fault indicator (X_BAT) value.
 * 
 * @param[in]   v_bat       battery voltage level (V)
 * @return      normalized battery voltage monitor fault indicator (X_BAT) value
 */
float x_bat_get_normalized(float v_bat) {
    /* Check if window size is reached */
    if(x_bat_data.cnt >= X_BAT_N) {
        /* Get oldest value from array (current index) */
        float old = x_bat_values[x_bat_index];
        /* Replace oldest value with new one */
        welford_replace(&x_bat_data, old, v_bat);
        /* Store value in array */
        x_bat_values[x_bat_index] = v_bat;
        /* Get next array index */
        x_bat_index = (x_bat_index+1) % X_BAT_N;
    } else {
        /* Fill array */
        for (uint8_t i=0; i<X_BAT_N; i++) {
            x_bat_values[i] = v_bat;
        }
        welford_add(&x_bat_data, v_bat);
        /* Manually set element count to N */
        x_bat_data.cnt = X_BAT_N;
        /* Set array index to 2. position */
        x_bat_index = 1;
    }
    /* Return normalized standard deviation as X_BAT (max: 1.0) */
    return fmin(1.0, welford_get_stddev(&x_bat_data) / X_BAT_MAX);
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
    /* Set array index to zero */
    x_art_index = 0;
    /* Initialize Welford data structure */
    welford_init(&x_art_data);
}

/*!
 * Get the normalized active runtime monitor fault indicator (X_ART) value.
 * 
 * @param[in]   t_art       length of the last active phase (ms)
 * @return      normalized active runtime monitor fault indicator (X_ART) value
 */
float x_art_get_normalized(uint32_t t_art) {
    /* Check if window size is reached */
    if(x_art_data.cnt >= X_ART_N) {
        /* Get oldest value from array (current index) */
        float old = x_art_values[x_art_index];
        /* Replace oldest value with new one */
        welford_replace(&x_art_data, old, t_art);
        /* Store value in array */
        x_art_values[x_art_index] = t_art;
        /* Get next array index */
        x_art_index = (x_art_index+1) % X_ART_N;
    } else {
        /* Fill array */
        for (uint8_t i=0; i<X_ART_N; i++) {
            x_art_values[i] = t_art;
        }
        welford_add(&x_art_data, t_art);
        /* Manually set element count to N */
        x_art_data.cnt = X_ART_N;
        /* Set array index to 2. position */
        x_art_index = 1;
    }
    /* Get standard deviation */
    float x_art_stddev = welford_get_stddev(&x_art_data);
    /* Return normalized X_ART depending on magnitude of difference (max: 1.0) */
    if(x_art_stddev>1.0) {
        return fmin(1.0, log10(x_art_stddev) / X_ART_MAX);
    } else {
        return 0.0;
    }
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
    x_rst = fmin(1.0, x_rst);
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
    return fmin(1.0, ((float)x_ic / X_IC_MAX));
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
        return fmin(1.0, ((adc_value-X_ADC_EXPECTED) / X_ADC_MAX));
    } else {
        return fmin(1.0, ((X_ADC_EXPECTED-adc_value) / X_ADC_MAX));
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
    return fmin(1.0, ((float)diff / X_USART_MAX));
}
