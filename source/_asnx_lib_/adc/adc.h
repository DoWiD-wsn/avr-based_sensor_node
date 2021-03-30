/**
 *  Header file for AVR ADC functionality.
 */

#ifndef _AVR_ADC_H_
#define _AVR_ADC_H_

/***** INCLUDES ***************************************************************/
#include <stdint.h>


/***** MACROS *****************************************************************/
/* Delays */
#define ADC_DELAY_CHANGE_REFERENCE      (25)
/* Available ADC channels */
#define ADC_CH0                         (0)
#define ADC_CH1                         (1)
#define ADC_CH2                         (2)
#define ADC_CH3                         (3)
#define ADC_CH4                         (4)
#define ADC_CH5                         (5)
#define ADC_CH6                         (6)
#define ADC_CH7                         (7)
#define ADC_TEMP                        (8)
#define ADC_1V1                         (14)
#define ADC_GND                         (15)
/* ADC Prescaler Options */
#define ADC_PRE_2                       (0)
#define ADC_PRE_4                       (2)
#define ADC_PRE_8                       (3)
#define ADC_PRE_16                      (4)
#define ADC_PRE_32                      (5)
#define ADC_PRE_64                      (6)
#define ADC_PRE_128                     (7)
/* ADC Reference Voltage Options */
#define ADC_REF_AREF                    (0)
#define ADC_REF_VCC                     (1)
#define ADC_REF_RES                     (2)
#define ADC_REF_INT1V1                  (3)


/***** GLOBAL VARIABLES *******************************************************/


/***** ENUMERATION ************************************************************/


/***** STRUCTURES *************************************************************/


/***** FUNCTION PROTOTYPES ****************************************************/
/* General (de)init */
void adc_init(uint8_t prescaler, uint8_t reference);
void adc_deinit(void);
/* Specific init */
void adc_enable(void);
void adc_disable(void);
void adc_set_channel(uint8_t channel);
void adc_set_prescaler(uint8_t prescaler);
void adc_set_reference(uint8_t reference);
/* Reading */
uint16_t adc_read(void);
uint16_t adc_read_channel(uint8_t channel);
/* Special functions */
float adc_read_vss(void);
float adc_read_temp(void);
/* Misc */
float adc_convert_temperature(uint16_t input);


/***** INLINE FUNCTIONS *******************************************************/


#endif // _AVR_ADC_H_
