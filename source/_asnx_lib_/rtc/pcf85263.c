/*****
 * @brief   ASN(x) PCR85263 RTC library
 *
 * Library to support the PCR85263 RTC module.
 *
 * @file    /_asnx_lib_/rtc/pcf85263.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1.0 $
 * @date    $Date: 2021/05/10 $
 *****/


/***** INCLUDES *******************************************************/
#include "pcf85263.h"
/*** ASNX LIB ***/
#include "i2c/i2c.h"


/***** LOCAL FUNCTION PROTOTYPES **************************************/
static inline uint8_t _bcd2dec(uint8_t value);
static inline uint8_t _dec2bcd(uint8_t value);


/***** FUNCTIONS ******************************************************/
/***
 * Convert a BCD-encoded value into a decimal value (0-99).
 * 
 * @param[in]   value   BCD-encoded value.
 * @return      Converted decimal value.
 ***/
static inline uint8_t _bcd2dec(uint8_t value) {
    /* Multiply high nibble by ten and add low nibble */
    return ((value&0xF0) >> 4) * 10 + (value&0x0F);
}


/***
 * Convert a decimal value into a BCD-encoded value (0-99).
 * 
 * @param[in]   value   Decimal value.
 * @return      Converted BCD-encoded value.
 ***/
static inline uint8_t _dec2bcd(uint8_t value) {
    /* Put number of tens to high nibble and unit digit to low nibble */
    return ((value/10) << 4) + (value%10);
}


/***
 * Initialize the PCF85263 RTC.
 * 
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_init(void) {
    /* Check if the device is available under the specified I2C address */
    if(i2c_is_available(PCF85263_I2C_ADDRESS) != I2C_RET_OK) {
        /* Device not accessible */
        return PCF85263_RET_NO_DEV;
    }
    /* Reset the device */
    return pcf85263_reset();
}


/***
 * Read a byte value from a given register.
 * 
 * @param[in]   reg     Register address
 * @param[out]  byte    Byte value read from register
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_read_reg(uint8_t reg, uint8_t* byte) {
    /* Try to read register via I2C */
    if(i2c_read_U8(PCF85263_I2C_ADDRESS, reg, byte) != I2C_RET_OK) {
        /* Write failed */
        *byte = 0x00;
        return PCF85263_RET_ERROR;
    }
    /* Read successful */
    return PCF85263_RET_OK;
}


/***
 * Write a byte value to a given register.
 * 
 * @param[in]   reg     Register address
 * @param[in]   byte    Byte value to be written to register
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_write_reg(uint8_t reg, uint8_t byte) {
    /* Try to write register via I2C */
    if(i2c_write_8(PCF85263_I2C_ADDRESS, reg, byte) != I2C_RET_OK) {
        /* Write failed */
        return PCF85263_RET_ERROR;
    }
    /* Write successful */
    return PCF85263_RET_OK;
}


/***
 * Start the RTC clock.
 * 
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_start(void) {
    /* Write 0 to the STOP bit of the stop_enable register (0x2E) */
    return pcf85263_write_reg(PCF85263_CTL_STOP_ENABLE, PCF85263_CTL_START);
}


/***
 * Stop the RTC clock.
 * 
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_stop(void) {
    /* Write 1 to the STOP bit of the stop_enable register (0x2E) */
    return pcf85263_write_reg(PCF85263_CTL_STOP_ENABLE, PCF85263_CTL_STOP);
}


/***
 * Perform a software reset (SR; also triggers CPR and CTS).
 * 
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_reset(void) {
    /* Request SR via the reset register (0x2F) */
    return pcf85263_write_reg(PCF85263_CTL_RESETS, (PCF85263_CTL_SR | PCF85263_CTL_RESETS_BITS));
}


/***
 * Perform a prescaler reset (CPR).
 * 
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_reset_prescaler(void) {
    /* Request CPR via the reset register (0x2F) */
    return pcf85263_write_reg(PCF85263_CTL_RESETS, (PCF85263_CTL_CPR | PCF85263_CTL_RESETS_BITS));
}


/***
 * Perform a timestamp reset (CTS).
 * 
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_reset_timestamp(void) {
    /* Request CTS via the reset register (0x2F) */
    return pcf85263_write_reg(PCF85263_CTL_RESETS, (PCF85263_CTL_CTS | PCF85263_CTL_RESETS_BITS));
}


/***
 * Read the RAM byte.
 * 
 * @param[out]  byte        RAM byte value read
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_read_ram(uint8_t* byte) {
    /* Read the RAM_byte register (0x2C) */
    return pcf85263_read_reg(PCF85263_CTL_RAM_BYTE, byte);
}


/***
 * Write the RAM byte.
 * 
 * @param[in]   byte        RAM byte value to be written
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_write_ram(uint8_t byte) {
    /* Write the RAM_byte register (0x2C) */
    return pcf85263_write_reg(PCF85263_CTL_RAM_BYTE, byte);
}


/***
 * Get the watchdog configuration byte.
 * 
 * @param[out]  value       Watchdog configuration byte value read
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_get_watchdog_cfg(uint8_t* value) {
    /* Get the watchdog registers (0x2D) */
    return pcf85263_read_reg(PCF85263_CTL_WATCHDOG, value);
}


/***
 * Set the watchdog configuration byte.
 * 
 * @param[in]   value       Watchdog configuration byte value to be written
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_set_watchdog_cfg(uint8_t value) {
    /* Set the watchdog registers (0x2D) */
    return pcf85263_write_reg(PCF85263_CTL_WATCHDOG, value);
}


/***
 * Read the offset value.
 * 
 * @param[out]  value       Offset value read
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_read_offset(uint8_t* value) {
    /* Get the offset register (0x24) */
    return pcf85263_read_reg(PCF85263_CTL_OFFSET, value);
}


/***
 * Write the offset value.
 * 
 * @param[in]   value       Offset value to be written
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_write_offset(uint8_t value) {
    /* Set the offset registers (0x24) */
    return pcf85263_write_reg(PCF85263_CTL_OFFSET, value);
}


/***
 * Get the oscillator register value.
 * 
 * @param[out]  value       Register value read
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_get_oscillator(uint8_t* value) {
    /* Get the oscillator control registers (0x25) */
    return pcf85263_read_reg(PCF85263_CTL_OSCILLATOR, value);
}


/***
 * Set the oscillator register value.
 * 
 * @param[in]   value       Register value to be written
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_set_oscillator(uint8_t value) {
    /* Set the oscillator control registers (0x25) */
    return pcf85263_write_reg(PCF85263_CTL_OSCILLATOR, value);
}


/***
 * Get the battery switch control register value.
 * 
 * @param[out]  value       Register value read
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_get_batteryswitch(uint8_t* value) {
    /* Get the battery switch control registers (0x26) */
    return pcf85263_read_reg(PCF85263_CTL_BATTERY_SWITCH, value);
}


/***
 * Set the battery switch control register value.
 * 
 * @param[in]   value       Register value to be written
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_set_batteryswitch(uint8_t value) {
    /* Set the battery switch control registers (0x26) */
    return pcf85263_write_reg(PCF85263_CTL_BATTERY_SWITCH, value);
}


/***
 * Get the pin IO control register value.
 * 
 * @param[out]  value       Register value read
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_get_pin_io(uint8_t* value) {
    /* Get the pin IO control registers (0x27) */
    return pcf85263_read_reg(PCF85263_CTL_PIN_IO, value);
}


/***
 * Set the pin IO control register value.
 * 
 * @param[in]   value       Register value to be written
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_set_pin_io(uint8_t value) {
    /* Set the pin IO control registers (0x27) */
    return pcf85263_write_reg(PCF85263_CTL_PIN_IO, value);
}


/***
 * Get the function control register value.
 * 
 * @param[out]  value       Register value read
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_get_function(uint8_t* value) {
    /* Get the function control registers (0x28) */
    return pcf85263_read_reg(PCF85263_CTL_FUNCTION, value);
}


/***
 * Set the function control register value.
 * 
 * @param[in]   value       Register value to be written
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_set_function(uint8_t value) {
    /* Set the function control registers (0x28) */
    return pcf85263_write_reg(PCF85263_CTL_FUNCTION, value);
}


/***
 * Get the interrupt A control register value.
 * 
 * @param[out]  value       Register value read
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_get_inta_en(uint8_t* value) {
    /* Get the interrupt A control registers (0x29) */
    return pcf85263_read_reg(PCF85263_CTL_INTA_ENABLE, value);
}


/***
 * Set the interrupt A control register value.
 * 
 * @param[in]   value       Register value to be written
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_set_inta_en(uint8_t value) {
    /* Set the interrupt A control registers (0x29) */
    return pcf85263_write_reg(PCF85263_CTL_INTA_ENABLE, value);
}


/***
 * Get the interrupt B control register value.
 * 
 * @param[out]  value       Register value read
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_get_intb_en(uint8_t* value) {
    /* Get the interrupt B control registers (0x2A) */
    return pcf85263_read_reg(PCF85263_CTL_INTB_ENABLE, value);
}


/***
 * Set the interrupt B control register value.
 * 
 * @param[in]   value       Register value to be written
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_set_intb_en(uint8_t value) {
    /* Set the interrupt A control registers (0x2A) */
    return pcf85263_write_reg(PCF85263_CTL_INTB_ENABLE, value);
}


/***
 * Get the flags register value.
 * 
 * @param[out]  value       Register value read
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_get_flags(uint8_t* value) {
    /* Get the flags registers (0x2B) */
    return pcf85263_read_reg(PCF85263_CTL_FLAGS, value);
}


/***
 * Set the flags register value.
 * 
 * @param[in]   value       Register value to be written
 * @return      OK in case of success; ERROR otherwise
 ***/
PCF85263_RET_t pcf85263_set_flags(uint8_t value) {
    /* Set the flags control registers (0x2B) */
    return pcf85263_write_reg(PCF85263_CTL_FLAGS, value);
}
