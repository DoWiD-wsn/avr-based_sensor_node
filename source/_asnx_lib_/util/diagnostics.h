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
/* Voltage-divider diagnostics */
/*! R_divider enable DDR register */
#define DIAG_RDEN_DDR                       DDRC
/*! R_divider enable PORT register */
#define DIAG_RDEN_PORT                      PORTC
/*! R_divider enable PIN register */
#define DIAG_RDEN_PIN                       PINC
/*! R_divider enable portpin index */
#define DIAG_RDEN_GPIO                      PC2

/* Reset-source decay parameters */
/*! decay rate (damping per update) */
#define DIAG_DECAY_RATE                     0.9
/*! decay minimum threshold */
#define DIAG_DECAY_MIN                      0.1
/*! decay maximum threshold */
#define DIAG_DECAY_MAX                      25.0
/*! EEPROM address */
#define DIAG_DECAY_ADDRESS                  0x00
/*! WDT increment value */
#define DIAG_DECAY_WDRF                     8.0
/*! BOD increment value */
#define DIAG_DECAY_BORF                     4.0
/*! EXT increment value */
#define DIAG_DECAY_EXTRF                    2.0
/*! PWR increment value */
#define DIAG_DECAY_PORF                     1.0


/***** FUNCTION PROTOTYPES ********************************************/
/* voltage-divider setup */
void diag_init(void);
void diag_enable(void);
void diag_disable(void);
/* voltage divider reading */
uint16_t diag_adc_check(void);
float diag_read_vcc(void);
float diag_read_vbat(void);
/* reset-source handling */
void diag_rsource_reset(void);
void diag_rsource_set(float value);
float diag_rsource_update(uint8_t mcusr);

#endif // _ASNX_DIAG_H_
