/*!
 * @brief   ASN(x) ADC library -- header file
 *
 * Library to support the use of the ADC.
 *
 * @file    /_asnx_lib_/adc/adc.h
 * @author  Dominik Widhalm
 * @version 1.2.3
 * @date    2022/01/10
 */

#ifndef _ASNX_ADC_H_
#define _ASNX_ADC_H_

/***** INCLUDES ***************************************************************/
/*** STD ***/
#include <stdint.h>
/*** AVR ***/
#include <avr/io.h>
#include <util/delay.h>


/***** MACROS *****************************************************************/
/*! Delay after changing the reference source [us] (min. 125us) */
#define ADC_DELAY_CHANGE_REFERENCE      (150)


/***** ENUMERATION ************************************************************/
/*! Enumeration for the available ADC input options */
typedef enum {
    /* Single ended input */
    ADC_CH0             = 0x00,     /**< Single ended input ADC0 */
    ADC_CH1             = 0x01,     /**< Single ended input ADC1 */
    ADC_CH2             = 0x02,     /**< Single ended input ADC2 */
    ADC_CH3             = 0x03,     /**< Single ended input ADC3 */
    ADC_CH4             = 0x04,     /**< Single ended input ADC4 */
    ADC_CH5             = 0x05,     /**< Single ended input ADC5 */
    ADC_CH6             = 0x06,     /**< Single ended input ADC6 */
    ADC_CH7             = 0x07,     /**< Single ended input ADC7 */
    /* Differential input (variable gain) */
    ADC_DIFF_0_0_X10    = 0x08,     /**< Differential input ADC0 - ADC0 (10x) */
    ADC_DIFF_1_0_X10    = 0x09,     /**< Differential input ADC1 - ADC0 (10x) */
    ADC_DIFF_0_0_X200   = 0x0A,     /**< Differential input ADC0 - ADC0 (200x) */
    ADC_DIFF_1_0_X200   = 0x0B,     /**< Differential input ADC1 - ADC0 (200x) */
    ADC_DIFF_2_2_X10    = 0x0C,     /**< Differential input ADC2 - ADC2 (10x) */
    ADC_DIFF_3_2_X10    = 0x0D,     /**< Differential input ADC3 - ADC2 (10x) */
    ADC_DIFF_2_2_X200   = 0x0E,     /**< Differential input ADC2 - ADC2 (200x) */
    ADC_DIFF_3_2_X200   = 0x0F,     /**< Differential input ADC3 - ADC2 (200x) */
    /* Differential input (no gain) */
    ADC_DIFF_0_1        = 0x10,     /**< Differential input ADC0 - ADC1 */
    ADC_DIFF_1_1        = 0x11,     /**< Differential input ADC1 - ADC1 */
    ADC_DIFF_2_1        = 0x12,     /**< Differential input ADC2 - ADC1 */
    ADC_DIFF_3_1        = 0x13,     /**< Differential input ADC3 - ADC1 */
    ADC_DIFF_4_1        = 0x14,     /**< Differential input ADC4 - ADC1 */
    ADC_DIFF_5_1        = 0x15,     /**< Differential input ADC5 - ADC1 */
    ADC_DIFF_6_1        = 0x16,     /**< Differential input ADC6 - ADC1 */
    ADC_DIFF_7_1        = 0x17,     /**< Differential input ADC7 - ADC1 */
    ADC_DIFF_0_2        = 0x18,     /**< Differential input ADC0 - ADC2 */
    ADC_DIFF_1_2        = 0x19,     /**< Differential input ADC1 - ADC2 */
    ADC_DIFF_2_2        = 0x1A,     /**< Differential input ADC2 - ADC2 */
    ADC_DIFF_3_2        = 0x1B,     /**< Differential input ADC3 - ADC2 */
    ADC_DIFF_4_2        = 0x1C,     /**< Differential input ADC4 - ADC2 */
    ADC_DIFF_5_2        = 0x1D,     /**< Differential input ADC5 - ADC2 */
    /* Fixed voltage inputs */
    ADC_1V1             = 0x1E,     /**< Fixed 1.1V (Vbg) input */
    ADC_GND             = 0x1F      /**< Fixed GND input */
} ADC_INPUT_t;


/*! Enumeration for the available ADC prescaler options */
typedef enum {
    ADC_ADPS_2          = 0x00,     /**< Division factor 2 */
    ADC_ADPS_4          = 0x02,     /**< Division factor 4 */
    ADC_ADPS_8          = 0x03,     /**< Division factor 8 */
    ADC_ADPS_16         = 0x04,     /**< Division factor 16 */
    ADC_ADPS_32         = 0x05,     /**< Division factor 32 */
    ADC_ADPS_64         = 0x06,     /**< Division factor 64 */
    ADC_ADPS_128        = 0x07      /**< Division factor 128 */
} ADC_PRESCALER_t;

/*! Enumeration for the available ADC reference voltage options */
typedef enum {
    ADC_REFS_AREF       = 0x00,     /**< AREF, internal Vref turned off */
    ADC_REFS_VCC        = 0x01,     /**< AVCC with external capacitor at AREF pin */
    ADC_REFS_INT1V1     = 0x02,     /**< Internal 1.1V Voltage Reference with external capacitor at AREF pin */
    ADC_REFS_INT2V56    = 0x03      /**< Internal 2.56V Voltage Reference with external capacitor at AREF pin */
} ADC_AREF_t;


/***** FUNCTION PROTOTYPES ****************************************************/
/* General (de)init */
void adc_init(ADC_PRESCALER_t prescaler, ADC_AREF_t reference);
void adc_deinit(void);
/* Specific init */
void adc_enable(void);
void adc_disable(void);
void adc_disable_input(uint8_t channels);
void adc_set_input(ADC_INPUT_t input);
void adc_set_prescaler(ADC_PRESCALER_t prescaler);
void adc_set_reference(ADC_AREF_t reference);
/* Reading */
uint16_t adc_read(void);
uint16_t adc_read_input(ADC_INPUT_t input);
/* Special functions */
float adc_read_vcc(void);


#endif // _ASNX_ADC_H_
