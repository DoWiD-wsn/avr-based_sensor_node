/*!
 * @brief   ASN(x) self-diagnostics library -- header file
 *
 * Library to support the self-diagnostic measures of the ASN(x).
 *
 * @file    /_asnx_lib_/util/diagnostics.h
 * @author  Dominik Widhalm
 * @version 1.4.0
 * @date    2021/10/18
 */

#ifndef _ASNX_DIAG_H_
#define _ASNX_DIAG_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
#include <stddef.h>
/*** ASNX LIB ***/
#include "adc/adc.h"
#include "hw/hw.h"


/***** DEFINES ********************************************************/
/* Voltage-divider diagnostics */
/*! R_divider enable DDR register */
#define DIAG_RDEN_DDR                       DDRC
/*! R_divider enable PORT register */
#define DIAG_RDEN_PORT                      PORTC
/*! R_divider enable PIN register */
#define DIAG_RDEN_PIN                       PINC
/*! R_divider enable portpin index */
#define DIAG_RDEN_GPIO                      PC2


/***** FUNCTION PROTOTYPES ********************************************/
/* voltage-divider setup */
void diag_init(void);
void diag_enable(void);
void diag_disable(void);
/* voltage divider reading */
uint16_t diag_adc_check(void);
float diag_read_vcc(void);
float diag_read_vbat(void);

#endif // _ASNX_DIAG_H_
