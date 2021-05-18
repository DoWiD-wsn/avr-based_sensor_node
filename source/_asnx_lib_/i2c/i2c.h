/*****
 * @brief   ASN(x) I2C/TWI library
 *
 * Library to support the use of the I2C/TWI module.
 *
 * @file    /_asnx_lib_/i2c/i2c.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1.0 $
 * @date    $Date: 2021/05/10 $
 *****/

#ifndef _ASNX_I2C_H_
#define _ASNX_I2C_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdio.h>
#include <stdint.h>


/***** DEFINES ********************************************************/
/*** CPU frequency (F_CPU) ***/
#ifndef F_CPU
# warning "F_CPU not defined for \"i2c.h\""
# define F_CPU 4000000UL
#endif
/* I2C speed */
#define I2C_SCL_CLK         (100000UL)
/* I2C timeout */
#define I2C_WAIT_TIMEOUT    (250)
/* Delay between write and read [ms] */
#define I2C_WR_DELAY        (100)


/***** ENUMERATION ****************************************************/
/* Enumeration for the I2C function return values */
typedef enum {
    I2C_RET_ERROR = -1,
    I2C_RET_OK = 0
} I2C_RET_t;
/* Enumeration for the I2C data direction */
typedef enum {
    I2C_READ = 1,
    I2C_WRITE = 0
} I2C_DIR_t;


/***** FUNCTION PROTOTYPES ********************************************/
/* General */
void i2c_init(void);
void i2c_reset(void);
I2C_RET_t i2c_is_available(uint8_t addr);
/* Raw Write/Read */
I2C_RET_t i2c_write_raw(uint8_t addr, uint8_t value);
I2C_RET_t i2c_read_raw(uint8_t addr, uint8_t* value);

/*** 8-bit register addresses ***/
/* Write */
I2C_RET_t i2c_write_block(uint8_t addr, uint8_t reg, uint8_t* value, uint8_t len);
I2C_RET_t i2c_write_8(uint8_t addr, uint8_t reg, uint8_t value);
I2C_RET_t i2c_write_16(uint8_t addr, uint8_t reg, uint16_t value);
/* Read */
I2C_RET_t i2c_read_block(uint8_t addr, uint8_t reg, uint8_t* value, uint8_t len);
I2C_RET_t i2c_read_U8(uint8_t addr, uint8_t reg, uint8_t* value);
I2C_RET_t i2c_read_S8(uint8_t addr, uint8_t reg, int8_t* value);
I2C_RET_t i2c_read_U16LE(uint8_t addr, uint8_t reg, uint16_t* value);
I2C_RET_t i2c_read_U16BE(uint8_t addr, uint8_t reg, uint16_t* value);
I2C_RET_t i2c_read_S16LE(uint8_t addr, uint8_t reg, int16_t* value);
I2C_RET_t i2c_read_S16BE(uint8_t addr, uint8_t reg, int16_t* value);

/*** 16-bit register addresses ***/
/* Write */
I2C_RET_t i2c_write16_block(uint8_t addr, uint16_t reg, uint8_t* value, uint8_t len);
I2C_RET_t i2c_write16_8(uint8_t addr, uint16_t reg, uint8_t value);
I2C_RET_t i2c_write16_16(uint8_t addr, uint16_t reg, uint16_t value);
/* Read */
I2C_RET_t i2c_read16_block(uint8_t addr, uint16_t reg, uint8_t* value, uint8_t len);
I2C_RET_t i2c_read16_U8(uint8_t addr, uint16_t reg, uint8_t* value);
I2C_RET_t i2c_read16_S8(uint8_t addr, uint16_t reg, int8_t* value);
I2C_RET_t i2c_read16_U16LE(uint8_t addr, uint16_t reg, uint16_t* value);
I2C_RET_t i2c_read16_U16BE(uint8_t addr, uint16_t reg, uint16_t* value);
I2C_RET_t i2c_read16_S16LE(uint8_t addr, uint16_t reg, int16_t* value);
I2C_RET_t i2c_read16_S16BE(uint8_t addr, uint16_t reg, int16_t* value);


#endif // _ASNX_I2C_H_
