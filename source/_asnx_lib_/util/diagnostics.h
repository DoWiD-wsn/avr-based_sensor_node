/*!
 * @brief   ASN(x) self-diagnostics library -- header file
 *
 * Library to support the self-diagnostic measures of the ASN(x).
 *
 * @file    /_asnx_lib_/util/diagnostics.h
 * @author  Dominik Widhalm
 * @version 1.3.0
 * @date    2021/08/09
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
/*! R_divider enable DDR register */
#define RDEN_DDR                            DDRC
/*! R_divider enable PORT register */
#define RDEN_PORT                           PORTC
/*! R_divider enable PIN register */
#define RDEN_PIN                            PINC
/*! R_divider enable portpin index */
#define RDEN_GPIO                           PC2


/***** FUNCTION PROTOTYPES ********************************************/
void diag_init(void);
void diag_enable(void);
void diag_disable(void);

uint16_t diag_adc_check(void);
float diag_read_vcc(void);
float diag_read_vbat(void);

#endif // _ASNX_DIAG_H_
