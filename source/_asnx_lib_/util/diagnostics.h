/*!
 * @brief   ASN(x) self-diagnostics library -- header file
 *
 * Library to support the self-diagnostic measures of the ASN(x).
 *
 * @file    /_asnx_lib_/util/diagnostics.h
 * @author  Dominik Widhalm
 * @version 1.4.2
 * @date    2022/01/25
 */

#ifndef _ASNX_DIAG_H_
#define _ASNX_DIAG_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
#include <stddef.h>
/*** AVR ***/
#include <util/delay.h>
/*** ASNX LIB ***/
#include "adc/adc.h"
#include "hw/hw.h"
#include "sensors/jt103.h"


/***** DEFINES ********************************************************/
/* ASN(x) hardware revision (minor) */
#ifndef ASNX_VERSION_MINOR
#  define ASNX_VERSION_MINOR                4
#endif

/* Voltage-divider diagnostics */
/*! R_divider enable DDR register */
#define DIAG_RDEN_DDR                       DDRC
/*! R_divider enable PORT register */
#define DIAG_RDEN_PORT                      PORTC
/*! R_divider enable PIN register */
#define DIAG_RDEN_PIN                       PINC
/*! R_divider enable portpin index */
#define DIAG_RDEN_GPIO                      PC2

/* ADC channel assignment */
/*! ADC self-check channel */
#define DIAG_ADC_CH                         ADC_CH0
#if ASNX_VERSION_MINOR>0
/*! Battery voltage ADC channel */
#  define DIAG_VBAT_CH                      ADC_CH1
/*! MCU surface temperature ADC channel */
#  define DIAG_TMCU_CH                      ADC_CH2
#else
/*! Battery voltage ADC channel */
#  define DIAG_VBAT_CH                      ADC_CH2
/*! MCU surface temperature ADC channel */
#  define DIAG_TMCU_CH                      ADC_CH1
#endif

/* Battery SoC */
/*! Battery maximum voltage */
#define DIAG_VBAT_MAX                       (3.2)
/*! Battery minimum voltage */
#define DIAG_VBAT_MIN                       (1.8)
/*! Battery voltage range */
#define DIAG_VBAT_RANGE                     (DIAG_VBAT_MAX - DIAG_VBAT_MIN)


/***** FUNCTION PROTOTYPES ********************************************/
/* voltage-divider setup */
void diag_init(void);
void diag_enable(void);
void diag_disable(void);
/* voltage divider reading */
uint16_t diag_adc_check(void);
float diag_vcc_read(void);
float diag_vbat_read(float vcc);
uint8_t diag_vbat_soc(float vbat);
float diag_tsurface_read(void);

#endif // _ASNX_DIAG_H_
