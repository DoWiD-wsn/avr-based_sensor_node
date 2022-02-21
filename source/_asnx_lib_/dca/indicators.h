/*!
 * @brief   ASN(x) fault indicator library -- header file
 *
 * Implementation of the fault indicators available on the ASN(x).
 *
 * @file    /_asnx_lib_/dca/indicators.h
 * @author  Dominik Widhalm
 * @version 2.0.0
 * @date    2022/01/25
 */

#ifndef _ASNX_INDICATORS_H_
#define _ASNX_INDICATORS_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
#include <stddef.h>
#include <math.h>
/*** AVR ***/
#include <avr/eeprom.h>
#include <avr/io.h>
/*** ASNX LIB ***/
#include "util/diagnostics.h"


/***** DEFINES ********************************************************/
/* Node temperature monitor (X_NT) */
/*! maximum deviation for normalization (degrees Celsius) */
#define X_NT_MAX                            5.0
/* Supply voltage monitor (X_VS) */
/*! maximum value for normalization (volts) */
#define X_VS_MAX                            1.0
/* Battery voltage monitor (X_BAT) */
/*! maximum value for normalization (volts) */
#define X_BAT_MAX                           1.0
/*! number of consecutive measurements (N) */
#define X_BAT_N                             5
/* Active runtime monitor (X_ART) */
/*! maximum value for change in magnitude */
#define X_ART_MAX                           5.0
/*! number of consecutive measurements (N) */
#define X_ART_N                             5
/* Reset monitor (X_RST) */
/*! maximum value for normalization */
#define X_RST_MAX                           25.0
/*! decay rate (damping per update) */
#define X_RST_DECAY                         0.9
/*! EEPROM address */
#define X_RST_EEPROM                        0x00
/*! EEPROM update threshold */
#define X_RST_UPDATE                        0.05
/* Software incident counter (X_IC) */
/*! maximum value for normalization */
#define X_IC_MAX                            10.0
/*! increment value for "normal" incident */
#define X_IC_INC_NORM                       1
/*! increment value for "serious" incident */
#define X_IC_INC_SERIOUS                    3
/*! decrement value */
#define X_IC_DEC_NORM                       1
/*! threshold for maximum number of incidents in total */
#define X_IC_THRESHOLD                      10
/* ADC self-check (X_ADC) */
/*! maximum value for normalization */
#define X_ADC_MAX                           25.0
/*! expected conversion result */
#define X_ADC_EXPECTED                      511
/* USART self-check (X_USART) */
/*! maximum value for normalization */
#define X_USART_MAX                         5.0


/***** FUNCTION PROTOTYPES ********************************************/
/* General */
void indicators_init(void);
/* Node temperature monitor (X_NT) */
float x_nt_get_normalized(float t_mcu, float t_brd, float t_trx);
/* Supply voltage monitor (X_VS) */
float x_vs_get_normalized(float v_mcu, float v_trx);
/* Battery voltage monitor (X_BAT) */
void x_bat_reset(void);
float x_bat_get_normalized(float v_bat);
float x_bat_get_mean(void);
/* Active runtime monitor (X_ART) */
void x_art_reset(void);
float x_art_get_normalized(uint16_t t_art);
/* Reset monitor (X_RST) */
void x_rst_reset(void);
void x_rst_set(float x_rst);
float x_rst_get_normalized(uint8_t mcusr);
/* Software incident counter (X_IC) */
void x_ic_reset(void);
void x_ic_inc(uint8_t value);
void x_ic_dec(uint8_t value);
uint16_t x_ic_get(void);
float x_ic_get_normalized(void);
/* ADC self-check (X_ADC) */
float x_adc_get_normalized(uint16_t adc_value);
/* USART self-check (X_USART) */
float x_usart_get_normalized(uint8_t* tx, uint8_t* rx, uint8_t len);

#endif // _ASNX_INDICATORS_H_
