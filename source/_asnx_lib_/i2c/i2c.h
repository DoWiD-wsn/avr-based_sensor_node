/*!
 * @brief   ASN(x) I2C (TWI) library -- header file
 *
 * Library to support the use of the I2C module.
 *
 * @file    /_asnx_lib_/i2c/i2c.h
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 */

#ifndef _ASNX_I2C_H_
#define _ASNX_I2C_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
/*** AVR ***/
#include <util/delay.h>
#include <util/twi.h>


/***** DEFINES ********************************************************/
/*! CPU frequency (F_CPU) */
#ifndef F_CPU
# warning "F_CPU not defined for \"i2c.h\""
# define F_CPU 4000000UL
#endif
/*! I2C speed */
#define I2C_SCL_CLK         (100000UL)
/*! I2C timeout */
#define I2C_WAIT_TIMEOUT    (250)
/*! Delay between write and read [ms] */
#define I2C_WR_DELAY        (100)


/***** ENUMERATION ****************************************************/
/*! Enumeration for the I2C function return values */
typedef enum {
    I2C_RET_ERROR = -1,     /**< ERROR return value */
    I2C_RET_OK = 0          /**< OK return value */
} I2C_RET_t;

/*! Enumeration for the endianess of data words */
typedef enum {
    I2C_ENDIAN_BIG = 1,     /**< big endian */
    I2C_ENDIAN_LITTLE = 0   /**< little endian */
} I2C_ENDIAN_t;

/*! Enumeration for the I2C data direction */
typedef enum {
    I2C_READ = 1,           /**< READ access */
    I2C_WRITE = 0           /**< WRITE access */
} I2C_DIR_t;


/***** FUNCTION PROTOTYPES ********************************************/
/* General */
void i2c_init(void);
void i2c_reset(void);
I2C_RET_t i2c_is_available(uint8_t addr);

/* Basic I2C functionality */
I2C_RET_t i2c_stop(void);
I2C_RET_t i2c_start(uint8_t addr, I2C_DIR_t dir);
I2C_RET_t i2c_start_wait(uint8_t addr, I2C_DIR_t dir);
I2C_RET_t i2c_put(uint8_t data);
I2C_RET_t i2c_get_ack(uint8_t* data);
I2C_RET_t i2c_get_nack(uint8_t* data);

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

/*** Read with endianess ***/
/* 8-bit register addresses */
I2C_RET_t i2c_read_U16(uint8_t addr, uint8_t reg, uint16_t* value, I2C_ENDIAN_t endian);
I2C_RET_t i2c_read_S16(uint8_t addr, uint8_t reg, int16_t* value, I2C_ENDIAN_t endian);
/* 16-bit register addresses */
I2C_RET_t i2c_read16_U16(uint8_t addr, uint16_t reg, uint16_t* value, I2C_ENDIAN_t endian);
I2C_RET_t i2c_read16_S16(uint8_t addr, uint16_t reg, int16_t* value, I2C_ENDIAN_t endian);

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
