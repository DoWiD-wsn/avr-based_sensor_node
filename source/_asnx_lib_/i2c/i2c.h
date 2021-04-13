/**
 *  Header file for AVR I2C functionality.
 * 
 *  Adapted taken from Peter Fleury
 *  -> $Id: i2cmaster.h,v 1.10 2005/03/06 22:39:57 Peter Exp $
 */

#ifndef _AVR_I2C_H_
#define _AVR_I2C_H_

/***** INCLUDES ***************************************************************/
#include <stdio.h>
#include <stdint.h>


/***** MACROS *****************************************************************/
/* General */
#ifndef F_CPU
# warning "F_CPU not defined for \"i2c.h\""
# define F_CPU 16000000UL
#endif
/* I2C speed */
#define I2C_SCL_CLK         (100000L)
/* I2C timeout */
#define I2C_WAIT_TIMEOUT    (250)
/* Delay between Write and Read */
#define I2C_WR_DELAY        (100)


/***** ENUMERATION ************************************************************/
/* Function return values */
typedef enum {
                I2C_RET_ERROR = -1,
                I2C_RET_OK = 0
             } i2c_ret_t;
/* Data direction */
typedef enum {
                I2C_READ = 1,
                I2C_WRITE = 0
             } i2c_dir_t;


/***** FUNCTION PROTOTYPES ****************************************************/
/* General */
void i2c_init(void);
/* Raw Write/Read */
i2c_ret_t i2c_write_raw(uint8_t addr, uint8_t value);
i2c_ret_t i2c_read_raw(uint8_t addr, uint8_t* value);
/*** 8-bit register addresses ***/
/* Write */
i2c_ret_t i2c_write_block(uint8_t addr, uint8_t reg, uint8_t* value, uint8_t len);
i2c_ret_t i2c_write_8(uint8_t addr, uint8_t reg, uint8_t value);
i2c_ret_t i2c_write_16(uint8_t addr, uint8_t reg, uint16_t value);
/* Read */
i2c_ret_t i2c_read_block(uint8_t addr, uint8_t reg, uint8_t* value, uint8_t len);
i2c_ret_t i2c_read_U8(uint8_t addr, uint8_t reg, uint8_t* value);
i2c_ret_t i2c_read_S8(uint8_t addr, uint8_t reg, int8_t* value);
i2c_ret_t i2c_read_U16LE(uint8_t addr, uint8_t reg, uint16_t* value);
i2c_ret_t i2c_read_U16BE(uint8_t addr, uint8_t reg, uint16_t* value);
i2c_ret_t i2c_read_S16LE(uint8_t addr, uint8_t reg, int16_t* value);
i2c_ret_t i2c_read_S16BE(uint8_t addr, uint8_t reg, int16_t* value);
/*** 16-bit register addresses ***/
/* Write */
i2c_ret_t i2c_write16_block(uint8_t addr, uint16_t reg, uint8_t* value, uint8_t len);
i2c_ret_t i2c_write16_8(uint8_t addr, uint16_t reg, uint8_t value);
i2c_ret_t i2c_write16_16(uint8_t addr, uint16_t reg, uint16_t value);
/* Read */
i2c_ret_t i2c_read16_block(uint8_t addr, uint16_t reg, uint8_t* value, uint8_t len);
i2c_ret_t i2c_read16_U8(uint8_t addr, uint16_t reg, uint8_t* value);
i2c_ret_t i2c_read16_S8(uint8_t addr, uint16_t reg, int8_t* value);
i2c_ret_t i2c_read16_U16LE(uint8_t addr, uint16_t reg, uint16_t* value);
i2c_ret_t i2c_read16_U16BE(uint8_t addr, uint16_t reg, uint16_t* value);
i2c_ret_t i2c_read16_S16LE(uint8_t addr, uint16_t reg, int16_t* value);
i2c_ret_t i2c_read16_S16BE(uint8_t addr, uint16_t reg, int16_t* value);


#endif // _AVR_I2C_H_