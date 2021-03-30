/**
 *  Header file for LM75 sensor.
 */

#ifndef _SENS_LM75_H_
#define _SENS_LM75_H_

/***** INCLUDES ***************************************************************/
#include <stdio.h>
#include <stdint.h>
#include "i2c/i2c.h"


/***** MACROS *****************************************************************/
/* I2C specific */
#define LM75_I2C_ADDRESS                (0x48)
/* I2C register addresses */
#define LM75_REG_TEMP                   (0x00)
#define LM75_REG_CONF                   (0x01)
#define LM75_REG_HYST                   (0x02)
#define LM75_REG_OS                     (0x03)
/* Configuration */
// shutdown
#define LM75_CONF_SHUTDOWN              (0)
// comparator/interrupt
#define LM75_CONF_COMP                  (1)
// OS polarity
#define LM75_CONF_POL                   (2)
// fault queue
#define LM75_CONF_QUEUE                 (3)
#define LM75_CONF_QUEUE_MASK            (0x18)
#define LM75_CONF_QUEUE_1               (0x00)
#define LM75_CONF_QUEUE_2               (0x01)
#define LM75_CONF_QUEUE_4               (0x02)
#define LM75_CONF_QUEUE_6               (0x03)


/***** ENUMERATION ************************************************************/
/* Function return values */
typedef enum {
                LM75_RET_ERROR = -1,
                LM75_RET_OK = 0
             } lm75_ret_t;


/***** FUNCTION PROTOTYPES ****************************************************/
void lm75_init(void);
/* Set */
lm75_ret_t lm75_set_config(uint8_t value);
lm75_ret_t lm75_set_hyst(float temp);
lm75_ret_t lm75_set_os(float temp);
/* Get */
lm75_ret_t lm75_get_config(uint8_t *value);
lm75_ret_t lm75_get_temperature(float *temp);
lm75_ret_t lm75_get_hyst(float *temp);
lm75_ret_t lm75_get_os(float *temp);


#endif // _SENS_LM75_H_
