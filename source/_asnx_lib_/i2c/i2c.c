/**
 *  Source file for AVR I2C functionality.
 */

/***** INCLUDES ***************************************************************/
/* AVR */
#include <util/delay.h>
#include <util/twi.h>
/* OWN */
#include "i2c.h"
#include "util/fuses.h"


/***** ENUMERATION ************************************************************/
/* Endianess */
typedef enum {
                I2C_ENDIAN_BIG = 1,
                I2C_ENDIAN_LITTLE = 0
             } i2c_endian_t;


/***** LOCAL FUNCTION PROTOTYPES **********************************************/
/*** Basic I2C functionality ***/
i2c_ret_t _i2c_stop(void);
i2c_ret_t _i2c_start(uint8_t addr, i2c_dir_t dir);
i2c_ret_t _i2c_start_wait(uint8_t addr, i2c_dir_t dir);
i2c_ret_t _i2c_put(uint8_t data);
i2c_ret_t _i2c_get_ack(uint8_t* data);
i2c_ret_t _i2c_get_nack(uint8_t* data);
/*** Read with endianess ***/
/* 8-bit register addresses */
i2c_ret_t _i2c_read_U16(uint8_t addr, uint8_t reg, uint16_t* value, i2c_endian_t endian);
i2c_ret_t _i2c_read_S16(uint8_t addr, uint8_t reg, int16_t* value, i2c_endian_t endian);
/* 16-bit register addresses */
i2c_ret_t _i2c_read16_U16(uint8_t addr, uint16_t reg, uint16_t* value, i2c_endian_t endian);
i2c_ret_t _i2c_read16_S16(uint8_t addr, uint16_t reg, int16_t* value, i2c_endian_t endian);


/***** FUNCTIONS **************************************************************/

/***** GENERAL *****/

/*
 * Initialization of the I2C master interface.
 */
void i2c_init(void) {
    /*** Initialize TWI clock ***/
    /* no prescaler */
    TWSR = 0;
    /* >10 for stable operation */
    TWBR = (uint8_t)(((fuses_get_fcpu()/I2C_SCL_CLK)-16.0)/2.0);
}


/***** BASIC *****/

/*
 * Stop I2C data transfer and release I2C bus.
 */
i2c_ret_t _i2c_stop(void) {
    /* Timeout counter */
    uint8_t timeout = 0;
    /* Send STOP condition */
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
    /* Wait until STOP condition is executed and bus released */
    timeout = 0;
    while(TWCR & _BV(TWSTO)){
        /* Check if timeout has been reached */
        if(timeout++ >= I2C_WAIT_TIMEOUT) {
            /* Something's wrong ... */
            return I2C_RET_ERROR;
        }
    }
    /* Return success */
    return I2C_RET_OK;
}


/*
 * Send I2C start condition, address and transfer direction.
 * 
 * uint8_t addr         address of I2C device
 * i2c_dir_t dir        transfer direction
 * return               I2C_RET_OK if device is accessible; I2C_RET_ERROR otherwise
 */
i2c_ret_t _i2c_start(uint8_t addr, i2c_dir_t dir) {
    /* Variable to temporary store status register value */
    uint8_t status_tmp;
    /* Timeout counter */
    uint8_t timeout = 0;
    /* Send START condition */
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
    /* Wait until transmission completed */
    timeout = 0;
    while(!(TWCR & _BV(TWINT))) {
        /* Check if timeout has been reached */
        if(timeout++ >= I2C_WAIT_TIMEOUT) {
            /* Something's wrong ... */
            return I2C_RET_ERROR;
        }
    }
    /* Check value of TWI status register */
    status_tmp = TW_STATUS;
    if((status_tmp != TW_START) && (status_tmp != TW_REP_START)) {
        /* Something's wrong ... */
        return I2C_RET_ERROR;
    }

    /* Send device address */
    TWDR = (addr<<1) | dir;
    TWCR = _BV(TWINT) | _BV(TWEN);
    /* Wail until transmission completed and ACK/NACK has been received */
    timeout = 0;
    while(!(TWCR & _BV(TWINT))) {
        /* Check if timeout has been reached */
        if(timeout++ >= I2C_WAIT_TIMEOUT) {
            /* Something's wrong ... */
            return I2C_RET_ERROR;
        }
    }
    /* Check value of TWI status register */
    status_tmp = TW_STATUS;
    if((status_tmp != TW_MT_SLA_ACK) && (status_tmp != TW_MR_SLA_ACK)) {
        /* Something's wrong ... */
        return I2C_RET_ERROR;
    }
    /* Return success */
    return I2C_RET_OK;
}


/*
 * Send I2C repeated start condition, address and transfer direction.
 * If device is busy, use ack polling to wait until device ready.
 * 
 * uint8_t addr         address of I2C device
 * i2c_dir_t dir        transfer direction
 * return               I2C_RET_OK if device is accessible; I2C_RET_ERROR otherwise
 */
i2c_ret_t _i2c_start_wait(uint8_t addr, i2c_dir_t dir) {
    /* Variable to temporary store status register value */
    uint8_t status_tmp;
    /* Try counter */
    uint8_t try = 0;
    /* Timeout counter */
    uint8_t timeout = 0;
    /* Try until success or timeout reached */
    while(1) {
        /* Check if timeout has been reached */
        if(try++ >= I2C_WAIT_TIMEOUT) {
            /* Something's wrong ... */
            return I2C_RET_ERROR;
        }
        /* Send START condition */
        TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
        /* Wait until transmission completed */
        timeout = 0;
        while(!(TWCR & _BV(TWINT))) {
            /* Check if timeout has been reached */
            if(timeout++ >= I2C_WAIT_TIMEOUT) {
                /* Something's wrong ... */
                return I2C_RET_ERROR;
            }
        }
        /* Check value of TWI status register */
        status_tmp = TW_STATUS;
        if((status_tmp != TW_START) && (status_tmp != TW_REP_START)) {
            /* Give it another try ... */
            continue;
        }
        /* Send device address */
        TWDR = (addr<<1) | dir;
        TWCR = _BV(TWINT) | _BV(TWEN);
        /* Wail until transmission completed */
        timeout = 0;
        while(!(TWCR & _BV(TWINT))) {
            /* Check if timeout has been reached */
            if(timeout++ >= I2C_WAIT_TIMEOUT) {
                /* Something's wrong ... */
                return I2C_RET_ERROR;
            }
        }
        /* Check value of TWI status register */
        status_tmp = TW_STATUS;
        if((status_tmp == TW_MT_SLA_NACK) || (status_tmp ==TW_MR_DATA_NACK)) {
            /* Device busy, send stop condition to terminate write operation */
            _i2c_stop();
            /* Give it another try ... */
            continue;
        }
        /* Looks like we've gone through ... */
        break;
    }
    /* Return success */
    return I2C_RET_OK;
}


/*
 * Write one byte to I2C device.
 * 
 * uint8_t data         byte to be transfered
 * return               I2C_RET_OK if write was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t _i2c_put(uint8_t data) {
    /* Timeout counter */
    uint8_t timeout = 0;
    /* Variable to temporary store status register value */
    uint8_t status_tmp;
    /* Send data to the previously addressed device */
    TWDR = data;
    TWCR = _BV(TWINT) | _BV(TWEN);
    /* Wait until transmission completed */
    timeout = 0;
    while(!(TWCR & _BV(TWINT))) {
        /* Check if timeout has been reached */
        if(timeout++ >= I2C_WAIT_TIMEOUT) {
            /* Something's wrong ... */
            return I2C_RET_ERROR;
        }
    }
    /* Check value of TWI status register */
    status_tmp = TW_STATUS;
    if(status_tmp != TW_MT_DATA_ACK) {
        /* Something's wrong ... */
        return I2C_RET_ERROR;
    }
    /* Return success */
    return I2C_RET_OK;
}


/*
 * Read one byte from the I2C device, request more data from device
 * 
 * uint8_t* data        address of byte to read the data to
 * return               I2C_RET_OK if read was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t _i2c_get_ack(uint8_t* data) {
    /* Timeout counter */
    uint8_t timeout = 0;
    /* Send a read request with acknowledgement (ack) */
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
    /* Wait until read is complete */
    timeout = 0;
    while(!(TWCR & _BV(TWINT))) {
        /* Check if timeout has been reached */
        if(timeout++ >= I2C_WAIT_TIMEOUT) {
            /* Something's wrong ... */
            return I2C_RET_ERROR;
        }
    }
    /* Copy the received byte */
    *data = TWDR;
    /* Return success */
    return I2C_RET_OK;
}


/*
 * Read one byte from the I2C device, read is followed by a stop condition
 * 
 * uint8_t* data        address of byte to read the data to
 * return               I2C_RET_OK if read was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t _i2c_get_nack(uint8_t* data) {
    /* Timeout counter */
    uint8_t timeout = 0;
    /* Send a read request without acknowledgement (nack) */
    TWCR = _BV(TWINT) | _BV(TWEN);
    /* Wait until read is complete */
    timeout = 0;
    while(!(TWCR & _BV(TWINT))) {
        /* Check if timeout has been reached */
        if(timeout++ >= I2C_WAIT_TIMEOUT) {
            /* Something's wrong ... */
            return I2C_RET_ERROR;
        }
    }
    /* Copy the received byte */
    *data = TWDR;
    /* Return success */
    return I2C_RET_OK;
}


/***** RAW WRITE/READ *****/

/*
 * Write 8-bit to a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint8_t value        byte value to be written
 * return               I2C_RET_OK if writing was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_write_raw(uint8_t addr, uint8_t value) {
    /* Start in writing mode */
    if(_i2c_start_wait(addr, I2C_WRITE) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Write value byte */
    if(_i2c_put(value) != I2C_RET_OK) {
        /* Write byte failed */
        return I2C_RET_ERROR;
    }
    /* Stop writing mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Stop failed */
        return I2C_RET_ERROR;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 8-bit from a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint8_t* value       address to write the byte to
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_read_raw(uint8_t addr, uint8_t* value) {
    /* Start in reading mode */
    if(_i2c_start_wait(addr, I2C_READ) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Read a byte */
    uint8_t tmp;
    /* Read with NACK */
    if(_i2c_get_nack(&tmp) == I2C_RET_OK) {
        /* Copy read byte */
        *value = tmp;
    } else {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Stop reading mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Stop failed */
        return I2C_RET_ERROR;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/***** 8-BIT REGISTER ADDRESSES *****/

/*** WRITE ***/

/*
 * Write a defined number of bytes to a register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint8_t reg          register address
 * uint8_t* value       address of first data byte
 * uint8_t len          number of bytes to be written
 * return               I2C_RET_OK if writing was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_write_block(uint8_t addr, uint8_t reg, uint8_t* value, uint8_t len) {
    /* Start in writing mode */
    if(_i2c_start_wait(addr, I2C_WRITE) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Write register address */
    if(_i2c_put(reg) != I2C_RET_OK) {
        /* Write address failed */
        return I2C_RET_ERROR;
    }
    /* Write "len" bytes */
    uint8_t cnt;
    for(cnt=0; cnt<len; cnt++) {
        /* Write cnt. byte */
        if(_i2c_put(value[cnt]) != I2C_RET_OK) {
            /* Write byte failed */
            return I2C_RET_ERROR;
        }
    }
    /* Stop writing mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Stop failed */
        return I2C_RET_ERROR;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Write 8-bit to a register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint8_t reg          register address
 * uint8_t value        8-bit value to be written
 * return               I2C_RET_OK if writing was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_write_8(uint8_t addr, uint8_t reg, uint8_t value) {
    /* Start in writing mode */
    if(_i2c_start_wait(addr, I2C_WRITE) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Write register address */
    if(_i2c_put(reg) != I2C_RET_OK) {
        /* Write address failed */
        return I2C_RET_ERROR;
    }
    /* Write value byte */
    if(_i2c_put(value) != I2C_RET_OK) {
        /* Write byte failed */
        return I2C_RET_ERROR;
    }
    /* Stop writing mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Stop failed */
        return I2C_RET_ERROR;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Write 16-bit to a register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint8_t reg          register address
 * uint16_t value       16-bit value to be written
 * return               I2C_RET_OK if writing was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_write_16(uint8_t addr, uint8_t reg, uint16_t value) {
    /* Start in writing mode */
    if(_i2c_start_wait(addr, I2C_WRITE) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Write register address */
    if(_i2c_put(reg) != I2C_RET_OK) {
        /* Write address failed */
        return I2C_RET_ERROR;
    }
    /* Write value lower byte */
    if(_i2c_put(value & 0x00FF) != I2C_RET_OK) {
        /* Write byte failed */
        return I2C_RET_ERROR;
    }
    /* Write value higher byte */
    if(_i2c_put((value & 0xFF00) >> 8) != I2C_RET_OK) {
        /* Write byte failed */
        return I2C_RET_ERROR;
    }
    /* Stop writing mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Stop failed */
        return I2C_RET_ERROR;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*** READ ***/

/*
 * Read a defined number of bytes from a register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint8_t reg          register address
 * uint8_t* value       address of first read byte
 * uint8_t len          number of bytes to be written
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_read_block(uint8_t addr, uint8_t reg, uint8_t* value, uint8_t len) {
    /* Start in writing mode */
    if(_i2c_start_wait(addr, I2C_WRITE) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Write register address */
    if(_i2c_put(reg) != I2C_RET_OK) {
        /* Write address failed */
        return I2C_RET_ERROR;
    }
    /* Stop writing mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Write byte failed */
        return I2C_RET_ERROR;
    }
    
    /* Give the device some time */
    _delay_ms(I2C_WR_DELAY);
    
    /* Restart in reading mode */
    if(_i2c_start_wait(addr, I2C_READ) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Read "len" bytes */
    uint8_t cnt,tmp;
    for(cnt=0; cnt<len; cnt++) {
        /* Check whether it is the last byte to read */
        if(cnt<(len-1)) {
            /* Read with ACK */
            if(_i2c_get_ack(&tmp) == I2C_RET_OK) {
                /* Copy read byte */
                value[cnt] = tmp;
            } else {
                /* Read failed */
                return I2C_RET_ERROR;
            }
        } else {
            /* Read with NACK */
            if(_i2c_get_nack(&tmp) == I2C_RET_OK) {
                /* Copy read byte */
                value[cnt] = tmp;
            } else {
                /* Read failed */
                return I2C_RET_ERROR;
            }
        }
    }
    /* Stop reading mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Stop failed */
        return I2C_RET_ERROR;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 8-bit (unsigned) from a register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint8_t reg          register address
 * uint8_t* value       address of 8-bit value to be read to
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_read_U8(uint8_t addr, uint8_t reg, uint8_t* value) {
/* Start in writing mode */
    if(_i2c_start_wait(addr, I2C_WRITE) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Write register address */
    if(_i2c_put(reg) != I2C_RET_OK) {
        /* Write address failed */
        return I2C_RET_ERROR;
    }
    /* Stop writing mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Write byte failed */
        return I2C_RET_ERROR;
    }
    
    /* Give the device some time */
    _delay_ms(I2C_WR_DELAY);
    
    /* Restart in reading mode */
    if(_i2c_start_wait(addr, I2C_READ) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Read 8 bit */
    uint8_t tmp;
    /* Read with NACK */
    if(_i2c_get_nack(&tmp) == I2C_RET_OK) {
        /* Copy read byte */
        *value = tmp;
    } else {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Stop reading mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Stop failed */
        return I2C_RET_ERROR;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 8-bit (signed) from a register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint8_t reg          register address
 * int8_t* value        address of 8-bit value to be read to
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_read_S8(uint8_t addr, uint8_t reg, int8_t* value) {
    uint8_t tmp;
    /* Read 8-bit unsigned */
    if(i2c_read_U8(addr, reg, &tmp) != I2C_RET_OK) {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Check if tmp sign bit is set */
    if(tmp > 127) {
        /* Negative value */
        *value = tmp-256;
    } else {
        /* Positive value */
        *value = tmp;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 16-bit (unsigned) from a register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint8_t reg          register address
 * uint16_t* value      address of 16-bit value to be read to
 * i2c_endian_t endian  endianess of the value
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t _i2c_read_U16(uint8_t addr, uint8_t reg, uint16_t* value, i2c_endian_t endian) {
    uint8_t tmp[2];
    /* Read two byte */
    if(i2c_read_block(addr, reg, tmp, 2) != I2C_RET_OK) {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Swap bytes if using big endian */
    if(endian == I2C_ENDIAN_BIG) {
        /* Big Endian */
        *value = ((uint16_t)tmp[0]<<8) | (uint16_t)tmp[1];
    } else {
        /* Little Endian */
        *value = ((uint16_t)tmp[1]<<8) | (uint16_t)tmp[0];
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 16-bit (signed) from a register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint8_t reg          register address
 * int16_t* value       address of 16-bit value to be read to
 * i2c_endian_t endian  endianess of the value
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t _i2c_read_S16(uint8_t addr, uint8_t reg, int16_t* value, i2c_endian_t endian) {
    uint16_t tmp;
    /* Read 16-bit unsigned */
    if(_i2c_read_U16(addr, reg, &tmp, endian) != I2C_RET_OK) {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Check if tmp sign bit is set */
    if(tmp > 32767) {
        /* Negative value */
        *value = tmp-65536;
    } else {
        /* Positive value */
        *value = tmp;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 16-bit (unsigned; little endian) from a register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint8_t reg          register address
 * uint16_t* value      address of 16-bit value to be read to
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_read_U16LE(uint8_t addr, uint8_t reg, uint16_t* value) {
    uint16_t tmp;
    /* Read 16-bit unsigned */
    if(_i2c_read_U16(addr, reg, &tmp, I2C_ENDIAN_LITTLE) != I2C_RET_OK) {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Copy value */
    *value = tmp;
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 16-bit (unsigned; big endian) from a register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint8_t reg          register address
 * uint16_t* value      address of 16-bit value to be read to
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_read_U16BE(uint8_t addr, uint8_t reg, uint16_t* value) {
    uint16_t tmp;
    /* Read 16-bit unsigned */
    if(_i2c_read_U16(addr, reg, &tmp, I2C_ENDIAN_BIG) != I2C_RET_OK) {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Copy value */
    *value = tmp;
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 16-bit (signed; little endian) from a register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint8_t reg          register address
 * int16_t* value       address of 16-bit value to be read to
 * i2c_endian_t endian  endianess of the value
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_read_S16LE(uint8_t addr, uint8_t reg, int16_t* value) {
    int16_t tmp;
    /* Read 16-bit signed */
    if(_i2c_read_S16(addr, reg, &tmp, I2C_ENDIAN_LITTLE) != I2C_RET_OK) {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Copy value */
    *value = tmp;
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 16-bit (signed; big endian) from a register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint8_t reg          register address
 * int16_t* value       address of 16-bit value to be read to
 * i2c_endian_t endian  endianess of the value
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_read_S16BE(uint8_t addr, uint8_t reg, int16_t* value) {
    int16_t tmp;
    /* Read 16-bit signed */
    if(_i2c_read_S16(addr, reg, &tmp, I2C_ENDIAN_BIG) != I2C_RET_OK) {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Copy value */
    *value = tmp;
    /* Return with success */
    return I2C_RET_OK;
}


/***** 16-BIT REGISTER ADDRESSES *****/

/*** WRITE ***/

/*
 * Write a defined number of bytes to a 16-bit register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint16_t reg         register address
 * uint8_t* value       address of first data byte
 * uint8_t len          number of bytes to be written
 * return               I2C_RET_OK if writing was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_write16_block(uint8_t addr, uint16_t reg, uint8_t* value, uint8_t len) {
    /* Start in writing mode */
    if(_i2c_start_wait(addr, I2C_WRITE) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Write register address low byte */
    if(_i2c_put(reg & 0x00FF) != I2C_RET_OK) {
        /* Write address failed */
        return I2C_RET_ERROR;
    }
    /* Write register address high byte */
    if(_i2c_put((reg & 0xFF00)>>8) != I2C_RET_OK) {
        /* Write address failed */
        return I2C_RET_ERROR;
    }
    /* Write "len" bytes */
    uint8_t cnt;
    for(cnt=0; cnt<len; cnt++) {
        /* Write cnt. byte */
        if(_i2c_put(value[cnt]) != I2C_RET_OK) {
            /* Write byte failed */
            return I2C_RET_ERROR;
        }
    }
    /* Stop writing mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Stop failed */
        return I2C_RET_ERROR;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Write 8-bit to a 16-bit register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint16_t reg         register address
 * uint8_t value        8-bit value to be written
 * return               I2C_RET_OK if writing was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_write16_8(uint8_t addr, uint16_t reg, uint8_t value) {
    /* Start in writing mode */
    if(_i2c_start_wait(addr, I2C_WRITE) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Write register address low byte */
    if(_i2c_put(reg & 0x00FF) != I2C_RET_OK) {
        /* Write address failed */
        return I2C_RET_ERROR;
    }
    /* Write register address high byte */
    if(_i2c_put((reg & 0xFF00)>>8) != I2C_RET_OK) {
        /* Write address failed */
        return I2C_RET_ERROR;
    }
    /* Write value byte */
    if(_i2c_put(value) != I2C_RET_OK) {
        /* Write byte failed */
        return I2C_RET_ERROR;
    }
    /* Stop writing mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Stop failed */
        return I2C_RET_ERROR;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Write 16-bit to a 16-bit register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint16_t reg          register address
 * uint16_t value       16-bit value to be written
 * return               I2C_RET_OK if writing was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_write16_16(uint8_t addr, uint16_t reg, uint16_t value) {
    /* Start in writing mode */
    if(_i2c_start_wait(addr, I2C_WRITE) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Write register address low byte */
    if(_i2c_put(reg & 0x00FF) != I2C_RET_OK) {
        /* Write address failed */
        return I2C_RET_ERROR;
    }
    /* Write register address high byte */
    if(_i2c_put((reg & 0xFF00)>>8) != I2C_RET_OK) {
        /* Write address failed */
        return I2C_RET_ERROR;
    }
    /* Write value lower byte */
    if(_i2c_put(value & 0x00FF) != I2C_RET_OK) {
        /* Write byte failed */
        return I2C_RET_ERROR;
    }
    /* Write value higher byte */
    if(_i2c_put((value & 0xFF00) >> 8) != I2C_RET_OK) {
        /* Write byte failed */
        return I2C_RET_ERROR;
    }
    /* Stop writing mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Stop failed */
        return I2C_RET_ERROR;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*** WRITE ***/

/*
 * Read a defined number of bytes from a 16-bit register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint16_t reg         register address
 * uint8_t* value       address of first read byte
 * uint8_t len          number of bytes to be written
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_read16_block(uint8_t addr, uint16_t reg, uint8_t* value, uint8_t len) {
    /* Start in writing mode */
    if(_i2c_start_wait(addr, I2C_WRITE) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Write register address low byte */
    if(_i2c_put(reg & 0x00FF) != I2C_RET_OK) {
        /* Write address failed */
        return I2C_RET_ERROR;
    }
    /* Write register address high byte */
    if(_i2c_put((reg & 0xFF00)>>8) != I2C_RET_OK) {
        /* Write address failed */
        return I2C_RET_ERROR;
    }
    /* Stop writing mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Write byte failed */
        return I2C_RET_ERROR;
    }
    
    /* Give the device some time */
    _delay_ms(I2C_WR_DELAY);
    
    /* Restart in reading mode */
    if(_i2c_start_wait(addr, I2C_READ) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Read "len" bytes */
    uint8_t cnt,tmp;
    for(cnt=0; cnt<len; cnt++) {
        /* Check whether it is the last byte to read */
        if(cnt<(len-1)) {
            /* Read with ACK */
            if(_i2c_get_ack(&tmp) == I2C_RET_OK) {
                /* Copy read byte */
                value[cnt] = tmp;
            } else {
                /* Read failed */
                return I2C_RET_ERROR;
            }
        } else {
            /* Read with NACK */
            if(_i2c_get_nack(&tmp) == I2C_RET_OK) {
                /* Copy read byte */
                value[cnt] = tmp;
            } else {
                /* Read failed */
                return I2C_RET_ERROR;
            }
        }
    }
    /* Stop reading mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Stop failed */
        return I2C_RET_ERROR;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 8-bit (unsigned) from a 16-bit register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint16_t reg         register address
 * uint8_t* value       address of 8-bit value to be read to
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_read16_U8(uint8_t addr, uint16_t reg, uint8_t* value) {
/* Start in writing mode */
    if(_i2c_start_wait(addr, I2C_WRITE) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Write register address low byte */
    if(_i2c_put(reg & 0x00FF) != I2C_RET_OK) {
        /* Write address failed */
        return I2C_RET_ERROR;
    }
    /* Write register address high byte */
    if(_i2c_put((reg & 0xFF00)>>8) != I2C_RET_OK) {
        /* Write address failed */
        return I2C_RET_ERROR;
    }
    /* Stop writing mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Write byte failed */
        return I2C_RET_ERROR;
    }
    
    /* Give the device some time */
    _delay_ms(I2C_WR_DELAY);
    
    /* Restart in reading mode */
    if(_i2c_start_wait(addr, I2C_READ) != I2C_RET_OK) {
        /* Start failed */
        return I2C_RET_ERROR;
    }
    /* Read 8 bit */
    uint8_t tmp;
    /* Read with NACK */
    if(_i2c_get_nack(&tmp) == I2C_RET_OK) {
        /* Copy read byte */
        *value = tmp;
    } else {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Stop reading mode */
    if(_i2c_stop() != I2C_RET_OK) {
        /* Stop failed */
        return I2C_RET_ERROR;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 8-bit (signed) from a 16-bit register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint16_t reg         register address
 * int8_t* value        address of 8-bit value to be read to
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_read16_S8(uint8_t addr, uint16_t reg, int8_t* value) {
    uint8_t tmp;
    /* Read 8-bit unsigned */
    if(i2c_read16_U8(addr, reg, &tmp) != I2C_RET_OK) {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Check if tmp sign bit is set */
    if(tmp > 127) {
        /* Negative value */
        *value = tmp-256;
    } else {
        /* Positive value */
        *value = tmp;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 16-bit (unsigned) from a 16-bit register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint16_t reg         register address
 * uint16_t* value      address of 16-bit value to be read to
 * i2c_endian_t endian  endianess of the value
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t _i2c_read16_U16(uint8_t addr, uint16_t reg, uint16_t* value, i2c_endian_t endian) {
    uint8_t tmp[2];
    /* Read two byte */
    if(i2c_read16_block(addr, reg, tmp, 2) != I2C_RET_OK) {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Swap bytes if using big endian */
    if(endian == I2C_ENDIAN_BIG) {
        /* Big Endian */
        *value = ((uint16_t)tmp[0]<<8) | (uint16_t)tmp[1];
    } else {
        /* Little Endian */
        *value = ((uint16_t)tmp[1]<<8) | (uint16_t)tmp[0];
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 16-bit (signed) from a 16-bit register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint16_t reg         register address
 * int16_t* value       address of 16-bit value to be read to
 * i2c_endian_t endian  endianess of the value
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t _i2c_read16_S16(uint8_t addr, uint16_t reg, int16_t* value, i2c_endian_t endian) {
    uint16_t tmp;
    /* Read 16-bit unsigned */
    if(_i2c_read16_U16(addr, reg, &tmp, endian) != I2C_RET_OK) {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Check if tmp sign bit is set */
    if(tmp > 32767) {
        /* Negative value */
        *value = tmp-65536;
    } else {
        /* Positive value */
        *value = tmp;
    }
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 16-bit (unsigned; little endian) from a 16-bit register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint16_t reg         register address
 * uint16_t* value      address of 16-bit value to be read to
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_read16_U16LE(uint8_t addr, uint16_t reg, uint16_t* value) {
    uint16_t tmp;
    /* Read 16-bit unsigned */
    if(_i2c_read16_U16(addr, reg, &tmp, I2C_ENDIAN_LITTLE) != I2C_RET_OK) {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Copy value */
    *value = tmp;
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 16-bit (unsigned; big endian) from a 16-bit register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint16_t reg         register address
 * uint16_t* value      address of 16-bit value to be read to
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_read16_U16BE(uint8_t addr, uint16_t reg, uint16_t* value) {
    uint16_t tmp;
    /* Read 16-bit unsigned */
    if(_i2c_read16_U16(addr, reg, &tmp, I2C_ENDIAN_BIG) != I2C_RET_OK) {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Copy value */
    *value = tmp;
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 16-bit (signed; little endian) from a 16-bit register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint16_t reg         register address
 * int16_t* value       address of 16-bit value to be read to
 * i2c_endian_t endian  endianess of the value
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_read16_S16LE(uint8_t addr, uint16_t reg, int16_t* value) {
    int16_t tmp;
    /* Read 16-bit signed */
    if(_i2c_read16_S16(addr, reg, &tmp, I2C_ENDIAN_LITTLE) != I2C_RET_OK) {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Copy value */
    *value = tmp;
    /* Return with success */
    return I2C_RET_OK;
}


/*
 * Read 16-bit (signed; big endian) from a 16-bit register of a given I2C address
 * 
 * uint8_t addr         address of I2C device
 * uint16_t reg         register address
 * int16_t* value       address of 16-bit value to be read to
 * i2c_endian_t endian  endianess of the value
 * return               I2C_RET_OK if reading was successful; I2C_RET_ERROR otherwise
 */
i2c_ret_t i2c_read16_S16BE(uint8_t addr, uint16_t reg, int16_t* value) {
    int16_t tmp;
    /* Read 16-bit signed */
    if(_i2c_read16_S16(addr, reg, &tmp, I2C_ENDIAN_BIG) != I2C_RET_OK) {
        /* Read failed */
        return I2C_RET_ERROR;
    }
    /* Copy value */
    *value = tmp;
    /* Return with success */
    return I2C_RET_OK;
}
