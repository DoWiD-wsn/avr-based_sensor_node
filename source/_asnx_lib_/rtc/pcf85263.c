/*!
 * @brief   ASN(x) PCR85263 RTC library -- source file
 *
 * Library to support the PCR85263 RTC module.
 *
 * @file    /_asnx_lib_/rtc/pcf85263.c
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 */


/***** INCLUDES *******************************************************/
#include "pcf85263.h"


/***** LOCAL FUNCTION PROTOTYPES **************************************/
static inline uint8_t _bcd2dec(uint8_t value);
static inline uint8_t _dec2bcd(uint8_t value);


/***** FUNCTIONS ******************************************************/
/*!
 * Convert a BCD-encoded value into a decimal value (0-99).
 * 
 * @param[in]   value   BCD-encoded value.
 * @return      Converted decimal value.
 */
static inline uint8_t _bcd2dec(uint8_t value) {
    /* Multiply high nibble by ten and add low nibble */
    return ((value&0xF0) >> 4) * 10 + (value&0x0F);
}


/*!
 * Convert a decimal value into a BCD-encoded value (0-99).
 * 
 * @param[in]   value   Decimal value.
 * @return      Converted BCD-encoded value.
 */
static inline uint8_t _dec2bcd(uint8_t value) {
    /* Put number of tens to high nibble and unit digit to low nibble */
    return ((value/10) << 4) + (value%10);
}


/*!
 * Initialize the PCF85263 RTC.
 * 
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_init(void) {
    /* Check if the device is available under the specified I2C address */
    if(i2c_is_available(PCF85263_I2C_ADDRESS) != I2C_RET_OK) {
        /* Device not accessible */
        return PCF85263_RET_NO_DEV;
    }
    /* Reset the device */
    if(pcf85263_reset() != PCF85263_RET_OK) {
        /* Device not accessible */
        return PCF85263_RET_ERROR;
    }
    /* Stop the device */
    if(pcf85263_stop() != PCF85263_RET_OK) {
        /* Device not accessible */
        return PCF85263_RET_ERROR;
    }
    /* Enable/disable the 100th seconds resolution */
#if PCF85263_100TH_SECONDS_ENABLE==1
    if(pcf85263_set_function(PCF85263_CTL_FUNC_100TH) != PCF85263_RET_OK) {
        /* Device not accessible */
        return PCF85263_RET_ERROR;
    }
#endif
    /* Set the 12h/24h configuration */
#if PCF85263_24H_MODE_ENABLE==0
    if(pcf85263_set_oscillator(PCF85263_CTL_OSC_12_24) != PCF85263_RET_OK) {
        /* Device not accessible */
        return PCF85263_RET_ERROR;
    }
#endif

    /* Initialization finished */
    return PCF85263_RET_OK;
}


/*!
 * Initialize the PCF85263 RTC as a wake-up source for the AVR MCU.
 * 
 * @param[in]   time    Time structure with the wake-up delay
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_init_wakeup_src(PCF85263_CNTTIME_t* time) {
    /* Temporary 8-bit register value */
    uint8_t reg = 0;
    
    /* Initialize the RTC */
    if(pcf85263_init() != PCF85263_RET_OK) {
        return PCF85263_RET_ERROR;
    }
    /* Disable the battery switch */
    if(pcf85263_set_batteryswitch(PCF85263_CTL_BATTERY_BSOFF) != PCF85263_RET_OK) {
        return PCF85263_RET_ERROR;
    }
    /* Disable CLK pin; INTA output */
    if(pcf85263_set_pin_io(PCF85263_CTL_CLKPM | PCF85263_CTL_INTAPM_INTA) != PCF85263_RET_OK) {
        return PCF85263_RET_ERROR;
    }
    /* Enable stop-watch mode (read first to get 100TH and STOPM bits) */
    if(pcf85263_get_function(&reg) != PCF85263_RET_OK) {
        return PCF85263_RET_ERROR;
    }
    reg |= PCF85263_CTL_FUNC_RTCM;
    if(pcf85263_set_function(reg) != PCF85263_RET_OK) {
        return PCF85263_RET_ERROR;
    }
    /* Set desired wake-up time */
    if(pcf85263_set_stw_alarm1(time) != PCF85263_RET_OK) {
        return PCF85263_RET_ERROR;
    }
    /* Enable the alarm */
    if(pcf85263_set_stw_alarm_enables(PCF85263_RTC_ALARM_MIN_A1E) != PCF85263_RET_OK) {
        return PCF85263_RET_ERROR;
    }
    /* Enable the alarm interrupt */
    if(pcf85263_set_inta_en(PCF85263_CTL_INTA_A1IEA) != PCF85263_RET_OK) {
        return PCF85263_RET_ERROR;
    }
    
    /* Initialization and configuration successful */
    return PCF85263_RET_OK;
}


/*!
 * Clear (reset) a given datetime structure (set all elements to 0).
 * 
 * @param[in]   data    Pointer to the datetime structure to be cleared.
 */
void pcf85263_clear_rtc_datetime(PCF85263_DATETIME_t* data) {
#if PCF85263_100TH_SECONDS_ENABLE==1
    data->msec10 = 0;
#endif
    data->seconds = 0;
    data->minutes = 0;
#if PCF85263_24H_MODE_ENABLE==1
    data->hours = 0;
#else
    data->ampm = 0;
    data->hours = 1;
#endif
    data->days = 1;
    data->wday = 0;
    data->months = 1;
    data->years = 0;
}


/*!
 * Clear (reset) a given time structure (set all elements to 0).
 * 
 * @param[in]   data    Pointer to the time structure to be cleared.
 */
void pcf85263_clear_stw_time(PCF85263_CNTTIME_t* data){
#if PCF85263_100TH_SECONDS_ENABLE==1
    data->msec10 = 0;
#endif
    data->seconds = 0;
    data->minutes = 0;
    data->hours = 0;
}


/*!
 * Read a byte value from a given register.
 * 
 * @param[in]   reg     Register address
 * @param[out]  byte    Byte value read from register
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_read_reg(uint8_t reg, uint8_t* byte) {
    /* Try to read register via I2C */
    if(i2c_read_U8(PCF85263_I2C_ADDRESS, reg, byte) != I2C_RET_OK) {
        /* Read failed */
        *byte = 0x00;
        return PCF85263_RET_ERROR;
    }
    /* Read successful */
    return PCF85263_RET_OK;
}


/*!
 * Write a byte value to a given register.
 * 
 * @param[in]   reg     Register address
 * @param[in]   byte    Byte value to be written to register
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_write_reg(uint8_t reg, uint8_t byte) {
    /* Try to write register via I2C */
    if(i2c_write_8(PCF85263_I2C_ADDRESS, reg, byte) != I2C_RET_OK) {
        /* Write failed */
        return PCF85263_RET_ERROR;
    }
    /* Write successful */
    return PCF85263_RET_OK;
}


/*!
 * Start the RTC clock.
 * 
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_start(void) {
    /* Write 0 to the STOP bit of the stop_enable register (0x2E) */
    return pcf85263_write_reg(PCF85263_CTL_STOP_ENABLE, PCF85263_CTL_START);
}


/*!
 * Stop the RTC clock.
 * 
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_stop(void) {
    /* Write 1 to the STOP bit of the stop_enable register (0x2E) */
    return pcf85263_write_reg(PCF85263_CTL_STOP_ENABLE, PCF85263_CTL_STOP);
}


/*!
 * Perform a software reset (SR; also triggers CPR and CTS).
 * 
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_reset(void) {
    /* Request SR via the reset register (0x2F) */
    return pcf85263_write_reg(PCF85263_CTL_RESETS, (PCF85263_CTL_SR | PCF85263_CTL_RESETS_BITS));
}


/*!
 * Perform a prescaler reset (CPR).
 * 
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_reset_prescaler(void) {
    /* Request CPR via the reset register (0x2F) */
    return pcf85263_write_reg(PCF85263_CTL_RESETS, (PCF85263_CTL_CPR | PCF85263_CTL_RESETS_BITS));
}


/*!
 * Perform a timestamp reset (CTS).
 * 
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_reset_timestamp(void) {
    /* Request CTS via the reset register (0x2F) */
    return pcf85263_write_reg(PCF85263_CTL_RESETS, (PCF85263_CTL_CTS | PCF85263_CTL_RESETS_BITS));
}


/*!
 * Read the EMON flag.
 * 
 * @param[out]  byte        EMON flag value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_EMON(uint8_t* value) {
    uint8_t tmp;
    /* Get the minutes registers (0x02) */
    if(pcf85263_read_reg(PCF85263_RTC_MINUTES, &tmp) != PCF85263_RET_OK) {
        /* Read failed */
        *value = 0;
        return PCF85263_RET_ERROR;
    }
    /* Check if EMON flag is set */
    if(tmp & PCF85263_FLAG_EMON) {
        *value = 1;
    } else {
        *value = 0;
    }
    /* Return success */
    return PCF85263_RET_OK;
}


/*!
 * Read the OS flag.
 * 
 * @param[out]  byte        OS flag value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_OS(uint8_t* value) {
    uint8_t tmp;
    /* Get the seconds registers (0x01) */
    if(pcf85263_read_reg(PCF85263_RTC_SECONDS, &tmp) != PCF85263_RET_OK) {
        /* Read failed */
        *value = 0;
        return PCF85263_RET_ERROR;
    }
    /* Check if OS flag is set */
    if(tmp & PCF85263_FLAG_OS) {
        *value = 1;
    } else {
        *value = 0;
    }
    /* Return success */
    return PCF85263_RET_OK;
}


/*!
 * Clear the OS flag.
 * 
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_clear_OS(void) {
    uint8_t tmp;
    /* Get the seconds registers (0x01) */
    if(pcf85263_read_reg(PCF85263_RTC_SECONDS, &tmp) != PCF85263_RET_OK) {
        /* Read failed */
        return PCF85263_RET_ERROR;
    }
    /* Clear flag in register value */
    tmp &= ~PCF85263_FLAG_OS;
    /* Write new register value to the seconds register (0x2C) */
    return pcf85263_write_reg(PCF85263_RTC_SECONDS, tmp);
}


/*!
 * Read the RAM byte.
 * 
 * @param[out]  byte        RAM byte value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_read_ram(uint8_t* byte) {
    /* Read the RAM_byte register (0x2C) */
    return pcf85263_read_reg(PCF85263_CTL_RAM_BYTE, byte);
}


/*!
 * Write the RAM byte.
 * 
 * @param[in]   byte        RAM byte value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_write_ram(uint8_t byte) {
    /* Write the RAM_byte register (0x2C) */
    return pcf85263_write_reg(PCF85263_CTL_RAM_BYTE, byte);
}


/*!
 * Get the watchdog configuration byte.
 * 
 * @param[out]  value       Watchdog configuration byte value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_watchdog_cfg(uint8_t* value) {
    /* Get the watchdog registers (0x2D) */
    return pcf85263_read_reg(PCF85263_CTL_WATCHDOG, value);
}


/*!
 * Set the watchdog configuration byte.
 * 
 * @param[in]   value       Watchdog configuration byte value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_watchdog_cfg(uint8_t value) {
    /* Set the watchdog registers (0x2D) */
    return pcf85263_write_reg(PCF85263_CTL_WATCHDOG, value);
}


/*!
 * Read the offset value.
 * 
 * @param[out]  value       Offset value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_read_offset(uint8_t* value) {
    /* Get the offset register (0x24) */
    return pcf85263_read_reg(PCF85263_CTL_OFFSET, value);
}


/*!
 * Write the offset value.
 * 
 * @param[in]   value       Offset value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_write_offset(uint8_t value) {
    /* Set the offset registers (0x24) */
    return pcf85263_write_reg(PCF85263_CTL_OFFSET, value);
}


/*!
 * Get the oscillator register value.
 * 
 * @param[out]  value       Register value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_oscillator(uint8_t* value) {
    /* Get the oscillator control registers (0x25) */
    return pcf85263_read_reg(PCF85263_CTL_OSCILLATOR, value);
}


/*!
 * Set the oscillator register value.
 * 
 * @param[in]   value       Register value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_oscillator(uint8_t value) {
    /* Set the oscillator control registers (0x25) */
    return pcf85263_write_reg(PCF85263_CTL_OSCILLATOR, value);
}


/*!
 * Get the battery switch control register value.
 * 
 * @param[out]  value       Register value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_batteryswitch(uint8_t* value) {
    /* Get the battery switch control registers (0x26) */
    return pcf85263_read_reg(PCF85263_CTL_BATTERY_SWITCH, value);
}


/*!
 * Set the battery switch control register value.
 * 
 * @param[in]   value       Register value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_batteryswitch(uint8_t value) {
    /* Set the battery switch control registers (0x26) */
    return pcf85263_write_reg(PCF85263_CTL_BATTERY_SWITCH, value);
}


/*!
 * Get the pin IO control register value.
 * 
 * @param[out]  value       Register value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_pin_io(uint8_t* value) {
    /* Get the pin IO control registers (0x27) */
    return pcf85263_read_reg(PCF85263_CTL_PIN_IO, value);
}


/*!
 * Set the pin IO control register value.
 * 
 * @param[in]   value       Register value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_pin_io(uint8_t value) {
    /* Set the pin IO control registers (0x27) */
    return pcf85263_write_reg(PCF85263_CTL_PIN_IO, value);
}


/*!
 * Get the function control register value.
 * 
 * @param[out]  value       Register value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_function(uint8_t* value) {
    /* Get the function control registers (0x28) */
    return pcf85263_read_reg(PCF85263_CTL_FUNCTION, value);
}


/*!
 * Set the function control register value.
 * 
 * @param[in]   value       Register value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_function(uint8_t value) {
    /* Set the function control registers (0x28) */
    return pcf85263_write_reg(PCF85263_CTL_FUNCTION, value);
}


/*!
 * Get the interrupt A control register value.
 * 
 * @param[out]  value       Register value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_inta_en(uint8_t* value) {
    /* Get the interrupt A control registers (0x29) */
    return pcf85263_read_reg(PCF85263_CTL_INTA_ENABLE, value);
}


/*!
 * Set the interrupt A control register value.
 * 
 * @param[in]   value       Register value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_inta_en(uint8_t value) {
    /* Set the interrupt A control registers (0x29) */
    return pcf85263_write_reg(PCF85263_CTL_INTA_ENABLE, value);
}


/*!
 * Get the interrupt B control register value.
 * 
 * @param[out]  value       Register value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_intb_en(uint8_t* value) {
    /* Get the interrupt B control registers (0x2A) */
    return pcf85263_read_reg(PCF85263_CTL_INTB_ENABLE, value);
}


/*!
 * Set the interrupt B control register value.
 * 
 * @param[in]   value       Register value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_intb_en(uint8_t value) {
    /* Set the interrupt A control registers (0x2A) */
    return pcf85263_write_reg(PCF85263_CTL_INTB_ENABLE, value);
}


/*!
 * Get the flags register value.
 * 
 * @param[out]  value       Register value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_flags(uint8_t* value) {
    /* Get the flags registers (0x2B) */
    return pcf85263_read_reg(PCF85263_CTL_FLAGS, value);
}


/*!
 * Set the flags register value.
 * 
 * @param[in]   value       Register value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_flags(uint8_t value) {
    /* Set the flags control registers (0x2B) */
    return pcf85263_write_reg(PCF85263_CTL_FLAGS, value);
}


/*!
 * Get the 100th of seconds register value.
 * 
 * @param[out]  seconds_100th   100th of seconds value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_100th_seconds(uint8_t* seconds_100th) {
    /* Get the 100th of seconds registers (0x00) */
    return pcf85263_read_reg(PCF85263_RTC_100TH_SECONDS, seconds_100th);
}


/*!
 * Set the 100th of seconds register value.
 * 
 * @param[in]   seconds_100th   100th of seconds value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_100th_seconds(uint8_t seconds_100th) {
    /* Set the 100th of seconds registers (0x00) */
    return pcf85263_write_reg(PCF85263_RTC_100TH_SECONDS, seconds_100th);
}


/*!
 * Get the seconds register value (excl. OS flag).
 * 
 * @param[out]  seconds     Seconds value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_seconds(uint8_t* seconds) {
    uint8_t tmp;
    /* Get the seconds registers (0x01) */
    if(pcf85263_read_reg(PCF85263_RTC_SECONDS, &tmp) != PCF85263_RET_OK) {
        /* Read failed */
        *seconds = 0x00;
        return PCF85263_RET_ERROR;
    }
    /* Convert BCD to decimal */
    *seconds = _bcd2dec(tmp & PCF85263_SECONDS_MASK);
    /* Return success */
    return PCF85263_RET_OK;
}


/*!
 * Set the seconds register value (excl. OS flag).
 * 
 * @param[in]   seconds     Seconds value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_seconds(uint8_t seconds) {
    /* Convert decimal to BCD */
    uint8_t tmp = _dec2bcd(seconds) & PCF85263_SECONDS_MASK;
    /* Set the seconds registers (0x01) */
    return pcf85263_write_reg(PCF85263_RTC_SECONDS, tmp);
}


/*!
 * Get the minutes register value (excl. EMON flag).
 * 
 * @param[out]  minutes     Minutes value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_minutes(uint8_t* minutes) {
    uint8_t tmp;
    /* Get the minutes registers (0x02) */
    if(pcf85263_read_reg(PCF85263_RTC_MINUTES, &tmp) != PCF85263_RET_OK) {
        /* Read failed */
        *minutes = 0x00;
        return PCF85263_RET_ERROR;
    }
    /* Convert BCD to decimal */
    *minutes = _bcd2dec(tmp & PCF85263_MINUTES_MASK);
    /* Return success */
    return PCF85263_RET_OK;
}


/*!
 * Set the minutes register value (excl. EMON flag).
 * 
 * @param[in]   minutes     Minutes value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_minutes(uint8_t minutes) {
    /* Convert decimal to BCD */
    uint8_t tmp = _dec2bcd(minutes) & PCF85263_MINUTES_MASK;
    /* Set the minutes registers (0x02) */
    return pcf85263_write_reg(PCF85263_RTC_MINUTES, tmp);
}


#if PCF85263_24H_MODE_ENABLE==1
/*!
 * Get the 24h hours register value (RTC mode).
 * 
 * @param[out]  hours       Hours value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_hours(uint8_t* hours) {
    uint8_t tmp;
    /* Get the hours registers (0x03) */
    if(pcf85263_read_reg(PCF85263_RTC_HOURS, &tmp) != PCF85263_RET_OK) {
        /* Read failed */
        *hours = 0x00;
        return PCF85263_RET_ERROR;
    }
    /* Convert BCD to decimal */
    *hours = _bcd2dec(tmp & PCF85263_HOURS24_MASK);
    /* Return success */
    return PCF85263_RET_OK;
}


/*!
 * Set the 24h hours register value (RTC mode).
 * 
 * @param[in]   hours       Hours value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_hours(uint8_t hours) {
    /* Convert decimal to BCD */
    uint8_t tmp = _dec2bcd(hours) & PCF85263_HOURS24_MASK;
    /* Set the hours registers (0x03) */
    return pcf85263_write_reg(PCF85263_RTC_HOURS, tmp);
}


#else
/*!
 * Get the 12h hours register value (RTC mode).
 * 
 * @param[out]  hours       Hours value read
 * @param[out]  ampm        AM/PM flag read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_hours(uint8_t* hours, uint8_t* ampm) {
    uint8_t tmp;
    /* Get the hours registers (0x03) */
    if(pcf85263_read_reg(PCF85263_RTC_HOURS, &tmp) != PCF85263_RET_OK) {
        /* Read failed */
        *hours = 0x00;
        return PCF85263_RET_ERROR;
    }
    /* Convert BCD to decimal */
    *hours = _bcd2dec(tmp & PCF85263_HOURS12_MASK);
    /* Get the AM/PM flag */
    if(tmp & PCF85263_HOURS12_AMPM) {
        *ampm = 1;
    } else {
        *ampm = 0;
    }
    /* Return success */
    return PCF85263_RET_OK;
}


/*!
 * Set the 12h hours register value (RTC mode).
 * 
 * @param[in]   hours       Hours value to be written
 * @param[in]   ampm        AM/PM flag to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_hours(uint8_t hours, uint8_t ampm) {
    /* Convert decimal to BCD */
    uint8_t tmp = _dec2bcd(hours) & PCF85263_HOURS12_AMPM;
    /* Check if AM/PM flag should be set */
    if(ampm!=0) {
        tmp |= PCF85263_HOURS12_AMPM;
    }
    /* Set the hours registers (0x03) */
    return pcf85263_write_reg(PCF85263_RTC_HOURS, tmp);
}
#endif


/*!
 * Get the days register value (RTC mode).
 * 
 * @param[out]  days        Days value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_days(uint8_t* days) {
    uint8_t tmp;
    /* Get the days registers (0x04) */
    if(pcf85263_read_reg(PCF85263_RTC_DAYS, &tmp) != PCF85263_RET_OK) {
        /* Read failed */
        *days = 0x00;
        return PCF85263_RET_ERROR;
    }
    /* Convert BCD to decimal */
    *days = _bcd2dec(tmp & PCF85263_DAYS_MASK);
    /* Return success */
    return PCF85263_RET_OK;
}


/*!
 * Set the days register value (RTC mode).
 * 
 * @param[in]   days        Days value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_days(uint8_t days) {
    /* Convert decimal to BCD */
    uint8_t tmp = _dec2bcd(days) & PCF85263_DAYS_MASK;
    /* Set the days registers (0x04) */
    return pcf85263_write_reg(PCF85263_RTC_DAYS, tmp);
}


/*!
 * Get the weekdays register value (RTC mode).
 * 
 * @param[out]  weekdays    Weekdays value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_weekdays(uint8_t* weekdays) {
    uint8_t tmp;
    /* Get the weekdays registers (0x05) */
    if(pcf85263_read_reg(PCF85263_RTC_WEEKDAYS, &tmp) != PCF85263_RET_OK) {
        /* Read failed */
        *weekdays = 0x00;
        return PCF85263_RET_ERROR;
    }
    /* Convert BCD to decimal */
    *weekdays = _bcd2dec(tmp & PCF85263_WEEKDAYS_MASK);
    /* Return success */
    return PCF85263_RET_OK;
}


/*!
 * Set the weekdays register value (RTC mode).
 * 
 * @param[in]   weekdays    Weekdays value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_weekdays(uint8_t weekdays) {
    /* Convert decimal to BCD */
    uint8_t tmp = _dec2bcd(weekdays) & PCF85263_WEEKDAYS_MASK;
    /* Set the weekdays registers (0x05) */
    return pcf85263_write_reg(PCF85263_RTC_WEEKDAYS, tmp);
}


/*!
 * Get the months register value (RTC mode).
 * 
 * @param[out]  months      Months value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_months(uint8_t* months) {
    uint8_t tmp;
    /* Get the months registers (0x06) */
    if(pcf85263_read_reg(PCF85263_RTC_MONTHS, &tmp) != PCF85263_RET_OK) {
        /* Read failed */
        *months = 0x00;
        return PCF85263_RET_ERROR;
    }
    /* Convert BCD to decimal */
    *months = _bcd2dec(tmp & PCF85263_MONTHS_MASK);
    /* Return success */
    return PCF85263_RET_OK;
}


/*!
 * Set the months register value (RTC mode).
 * 
 * @param[in]   months      Months value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_months(uint8_t months) {
    /* Convert decimal to BCD */
    uint8_t tmp = _dec2bcd(months) & PCF85263_MONTHS_MASK;
    /* Set the months registers (0x06) */
    return pcf85263_write_reg(PCF85263_RTC_MONTHS, tmp);
}


/*!
 * Get the years register value (RTC mode).
 * 
 * @param[out]  years       Years value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_years(uint8_t* years) {
    uint8_t tmp;
    /* Get the years registers (0x07) */
    if(pcf85263_read_reg(PCF85263_RTC_YEARS, &tmp) != PCF85263_RET_OK) {
        /* Read failed */
        *years = 0x00;
        return PCF85263_RET_ERROR;
    }
    /* Convert BCD to decimal */
    *years = _bcd2dec(tmp);
    /* Return success */
    return PCF85263_RET_OK;
}


/*!
 * Set the years register value (RTC mode).
 * 
 * @param[in]   years       Years value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_years(uint8_t years) {
    /* Convert decimal to BCD */
    uint8_t tmp = _dec2bcd(years);
    /* Set the years registers (0x07) */
    return pcf85263_write_reg(PCF85263_RTC_YEARS, tmp);
}


/*!
 * Read the current time and date (RTC mode).
 * 
 * @param[out]  data        Pointer to the date-time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_rtc_datetime(PCF85263_DATETIME_t* data) {
    /* Data array to be filled */
    uint8_t tmp[8] = {0};
    /* Read raw data */
#if PCF85263_100TH_SECONDS_ENABLE==1
    if(i2c_read_block(PCF85263_I2C_ADDRESS,PCF85263_RTC_100TH_SECONDS,tmp,8) != I2C_RET_OK) {
#else
    if(i2c_read_block(PCF85263_I2C_ADDRESS,PCF85263_RTC_SECONDS,&tmp[1],7) != I2C_RET_OK) {
#endif
        /* Reading failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Extract information from raw data */
#if PCF85263_100TH_SECONDS_ENABLE==1
    data->msec10    = _bcd2dec(tmp[0]);
#endif
    data->seconds   = _bcd2dec(tmp[1] & PCF85263_SECONDS_MASK);
    data->minutes   = _bcd2dec(tmp[2] & PCF85263_MINUTES_MASK);
#if PCF85263_24H_MODE_ENABLE==1
    data->hours     = _bcd2dec(tmp[3] & PCF85263_HOURS24_MASK);
#else
    data->ampm      = (tmp[3] & PCF85263_HOURS12_AMPM) ? 1 : 0;
    data->hours     = _bcd2dec(tmp[3] & PCF85263_HOURS12_MASK);
#endif
    data->days      = _bcd2dec(tmp[4] & PCF85263_DAYS_MASK);
    data->wday      = _bcd2dec(tmp[5] & PCF85263_WEEKDAYS_MASK);
    data->months    = _bcd2dec(tmp[6] & PCF85263_MONTHS_MASK);
    data->years     = _bcd2dec(tmp[7]);
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Write the time and date (RTC mode).
 * 
 * @param[in]   data        Pointer to the date-time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_rtc_datetime(PCF85263_DATETIME_t* data) {
    /* Temporary data array */
    uint8_t tmp[8] = {0};
    /* Write information into data array */
#if PCF85263_100TH_SECONDS_ENABLE==1
    tmp[0]  = _dec2bcd(data->msec10);
#endif
    tmp[1]  = _dec2bcd(data->seconds) & PCF85263_SECONDS_MASK;
    tmp[2]  = _dec2bcd(data->minutes) & PCF85263_MINUTES_MASK;
#if PCF85263_24H_MODE_ENABLE==1
    tmp[3]  = _dec2bcd(data->hours) & PCF85263_HOURS24_MASK;
#else
    tmp[3]  = (_dec2bcd(data->hours) & PCF85263_HOURS12_MASK;
    if(data->ampm == 1) {
        tmp[3] |= PCF85263_HOURS12_AMPM;
    }
#endif
    tmp[4]  = _dec2bcd(data->days) & PCF85263_DAYS_MASK;
    tmp[5]  = _dec2bcd(data->wday) & PCF85263_WEEKDAYS_MASK;
    tmp[6]  = _dec2bcd(data->months) & PCF85263_MONTHS_MASK;
    tmp[7]  = _dec2bcd(data->years);
    
    /* Write data to the device */
#if PCF85263_100TH_SECONDS_ENABLE==1
    if(i2c_write_block(PCF85263_I2C_ADDRESS,PCF85263_RTC_100TH_SECONDS,tmp,8) != I2C_RET_OK) {
#else
    if(i2c_write_block(PCF85263_I2C_ADDRESS,PCF85263_RTC_SECONDS,&tmp[1],7) != I2C_RET_OK) {
#endif
        /* Writing failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Get the hours xx_xx_00 register value (stop-watch mode).
 * 
 * @param[out]  hours       Hours value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_hours_xx_xx_00(uint8_t* hours) {
    uint8_t tmp;
    /* Get the hours xx_xx_00 registers (0x03) */
    if(pcf85263_read_reg(PCF85263_STW_HOURS_XX_XX_00, &tmp) != PCF85263_RET_OK) {
        /* Read failed */
        *hours = 0x00;
        return PCF85263_RET_ERROR;
    }
    /* Convert BCD to decimal */
    *hours = _bcd2dec(tmp);
    /* Return success */
    return PCF85263_RET_OK;
}


/*!
 * Set the hours xx_xx_00 register value (stop-watch mode).
 * 
 * @param[in]   hours       Hours value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_hours_xx_xx_00(uint8_t hours) {
    /* Convert decimal to BCD */
    uint8_t tmp = _dec2bcd(hours);
    /* Set the hours xx_xx_00 registers (0x03) */
    return pcf85263_write_reg(PCF85263_STW_HOURS_XX_XX_00, tmp);
}


/*!
 * Get the hours xx_00_xx register value (stop-watch mode).
 * 
 * @param[out]  hours       100-hours value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_hours_xx_00_xx(uint8_t* hours) {
    uint8_t tmp;
    /* Get the hours xx_00_xx registers (0x04) */
    if(pcf85263_read_reg(PCF85263_STW_HOURS_XX_00_XX, &tmp) != PCF85263_RET_OK) {
        /* Read failed */
        *hours = 0x00;
        return PCF85263_RET_ERROR;
    }
    /* Convert BCD to decimal */
    *hours = _bcd2dec(tmp);
    /* Return success */
    return PCF85263_RET_OK;
}


/*!
 * Set the hours xx_00_xx register value (stop-watch mode).
 * 
 * @param[in]   hours       100-hours value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_hours_xx_00_xx(uint8_t hours) {
    /* Convert decimal to BCD */
    uint8_t tmp = _dec2bcd(hours);
    /* Set the hours xx_00_xx registers (0x04) */
    return pcf85263_write_reg(PCF85263_STW_HOURS_XX_00_XX, tmp);
}


/*!
 * Get the hours 00_xx_xx register value (stop-watch mode).
 * 
 * @param[out]  hours       10-thousand-hours value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_hours_00_xx_xx(uint8_t* hours) {
    uint8_t tmp;
    /* Get the hours 00_xx_xx registers (0x05) */
    if(pcf85263_read_reg(PCF85263_STW_HOURS_00_XX_XX, &tmp) != PCF85263_RET_OK) {
        /* Read failed */
        *hours = 0x00;
        return PCF85263_RET_ERROR;
    }
    /* Convert BCD to decimal */
    *hours = _bcd2dec(tmp);
    /* Return success */
    return PCF85263_RET_OK;
}


/*!
 * Set the hours 00_xx_xx register value (stop-watch mode).
 * 
 * @param[in]   hours       10-thousand-hours value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_hours_00_xx_xx(uint8_t hours) {
    /* Convert decimal to BCD */
    uint8_t tmp = _dec2bcd(hours);
    /* Set the hours 00_xx_xx registers (0x05) */
    return pcf85263_write_reg(PCF85263_STW_HOURS_00_XX_XX, tmp);
}


/*!
 * Read the current time (stop-watch mode).
 * 
 * @param[out]  data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_stw_time(PCF85263_CNTTIME_t* data) {
    /* Data array to be filled */
    uint8_t tmp[6] = {0};
    /* Read raw data */
#if PCF85263_100TH_SECONDS_ENABLE==1
    if(i2c_read_block(PCF85263_I2C_ADDRESS,PCF85263_STW_100TH_SECONDS,tmp,6) != I2C_RET_OK) {
#else
    if(i2c_read_block(PCF85263_I2C_ADDRESS,PCF85263_STW_SECONDS,&tmp[1],5) != I2C_RET_OK) {
#endif
        /* Reading failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Extract information from raw data */
#if PCF85263_100TH_SECONDS_ENABLE==1
    data->msec10    = _bcd2dec(tmp[0]);
#endif
    data->seconds   = _bcd2dec(tmp[1] & PCF85263_SECONDS_MASK);
    data->minutes   = _bcd2dec(tmp[2] & PCF85263_MINUTES_MASK);
    data->hours     = _bcd2dec(tmp[3]);
    data->hours    += _bcd2dec(tmp[4]) * 100;
    data->hours    += _bcd2dec(tmp[5]) * 10000UL;
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Write the time (stop-watch mode).
 * 
 * @param[in]   data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_stw_time(PCF85263_CNTTIME_t* data) {
    /* Temporary data array */
    uint8_t tmp[6] = {0};
    /* Write information into data array */
#if PCF85263_100TH_SECONDS_ENABLE==1
    tmp[0]  = _dec2bcd(data->msec10);
#endif
    tmp[1]  = _dec2bcd(data->seconds) & PCF85263_SECONDS_MASK;
    tmp[2]  = _dec2bcd(data->minutes) & PCF85263_MINUTES_MASK;
    tmp[3]  = _dec2bcd(data->hours % 100);
    tmp[4]  = _dec2bcd((data->hours / 100) % 100);
    tmp[5]  = _dec2bcd((data->hours / 10000UL) % 100);
    
    /* Write data to the device */
#if PCF85263_100TH_SECONDS_ENABLE==1
    if(i2c_write_block(PCF85263_I2C_ADDRESS,PCF85263_STW_100TH_SECONDS,tmp,6) != I2C_RET_OK) {
#else
    if(i2c_write_block(PCF85263_I2C_ADDRESS,PCF85263_STW_SECONDS,&tmp[1],5) != I2C_RET_OK) {
#endif
        /* Writing failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Read the alarm enables register value.
 * 
 * @param[in]   value       Alarm enables value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_rtc_alarm_enables(uint8_t* value) {
    /* Get the alarm enables register (0x10) */
    return pcf85263_read_reg(PCF85263_RTC_ALARM_ENABLES, value);
}


/*!
 * Write the alarm enables register value.
 * 
 * @param[in]   value       Alarm enables value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_rtc_alarm_enables(uint8_t value) {
    /* Set the alarm enables registers (0x10) */
    return pcf85263_write_reg(PCF85263_RTC_ALARM_ENABLES, value);
}


/*!
 * Read the alarm1 date-time registers (RTC mode).
 * 
 * @param[out]  data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_rtc_alarm1(PCF85263_DATETIME_t* data) {
    /* Data array to be filled */
    uint8_t tmp[5] = {0};
    /* Read raw data */
    if(i2c_read_block(PCF85263_I2C_ADDRESS,PCF85263_RTC_SECOND_ALARM1,tmp,5) != I2C_RET_OK) {
        /* Reading failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Extract information from raw data */
#if PCF85263_100TH_SECONDS_ENABLE==1
    data->msec10    = 0;                    /**< not available for alarm1 */
#endif
    data->seconds   = _bcd2dec(tmp[0] & PCF85263_SECONDS_MASK);
    data->minutes   = _bcd2dec(tmp[1] & PCF85263_MINUTES_MASK);
#if PCF85263_24H_MODE_ENABLE==1
    data->hours     = _bcd2dec(tmp[2] & PCF85263_HOURS24_MASK);
#else
    data->ampm      = (tmp[2] & PCF85263_HOURS12_AMPM) ? 1 : 0;
    data->hours     = _bcd2dec(tmp[2] & PCF85263_HOURS12_MASK);
#endif
    data->days      = _bcd2dec(tmp[3] & PCF85263_DAYS_MASK);
    data->wday      = 0;                    /**< not available for alarm1 */
    data->months    = _bcd2dec(tmp[4] & PCF85263_MONTHS_MASK);
    data->years     = 0;                    /**< not available for alarm1 */
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Write the alarm1 date-time registers (RTC mode).
 * 
 * @param[in]   data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_rtc_alarm1(PCF85263_DATETIME_t* data) {
    /* Temporary data array */
    uint8_t tmp[5] = {0};
    /* Write information into data array */
    tmp[0]  = _dec2bcd(data->seconds) & PCF85263_SECONDS_MASK;
    tmp[1]  = _dec2bcd(data->minutes) & PCF85263_MINUTES_MASK;
#if PCF85263_24H_MODE_ENABLE==1
    tmp[2]  = _dec2bcd(data->hours) & PCF85263_HOURS24_MASK;
#else
    tmp[2]  = (_dec2bcd(data->hours) & PCF85263_HOURS12_MASK;
    if(data->ampm == 1) {
        tmp[2] |= PCF85263_HOURS12_AMPM;
    }
#endif
    tmp[3]  = _dec2bcd(data->days) & PCF85263_DAYS_MASK;
    tmp[4]  = _dec2bcd(data->months) & PCF85263_MONTHS_MASK;
    
    /* Write data to the device */
    if(i2c_write_block(PCF85263_I2C_ADDRESS,PCF85263_RTC_SECOND_ALARM1,tmp,5) != I2C_RET_OK) {
        /* Writing failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Read the alarm2 date-time registers (RTC mode).
 * 
 * @param[out]  data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_rtc_alarm2(PCF85263_DATETIME_t* data) {
    /* Data array to be filled */
    uint8_t tmp[3] = {0};
    /* Read raw data */
    if(i2c_read_block(PCF85263_I2C_ADDRESS,PCF85263_RTC_MINUTE_ALARM2,tmp,3) != I2C_RET_OK) {
        /* Reading failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Extract information from raw data */
#if PCF85263_100TH_SECONDS_ENABLE==1
    data->msec10    = 0;                    /**< not available for alarm2 */
#endif
    data->seconds   = 0;                    /**< not available for alarm2 */
    data->minutes   = _bcd2dec(tmp[0] & PCF85263_MINUTES_MASK);
#if PCF85263_24H_MODE_ENABLE==1
    data->hours     = _bcd2dec(tmp[1] & PCF85263_HOURS24_MASK);
#else
    data->ampm      = (tmp[1] & PCF85263_HOURS12_AMPM) ? 1 : 0;
    data->hours     = _bcd2dec(tmp[1] & PCF85263_HOURS12_MASK);
#endif
    data->days      = 0;                    /**< not available for alarm2 */
    data->wday      = _bcd2dec(tmp[2] & PCF85263_WEEKDAYS_MASK);
    data->months    = 0;                    /**< not available for alarm2 */
    data->years     = 0;                    /**< not available for alarm2 */
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Write the alarm2 date-time registers (RTC mode).
 * 
 * @param[in]   data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_rtc_alarm2(PCF85263_DATETIME_t* data) {
    /* Temporary data array */
    uint8_t tmp[3] = {0};
    /* Write information into data array */
    tmp[0]  = _dec2bcd(data->minutes) & PCF85263_MINUTES_MASK;
#if PCF85263_24H_MODE_ENABLE==1
    tmp[1]  = _dec2bcd(data->hours) & PCF85263_HOURS24_MASK;
#else
    tmp[1]  = (_dec2bcd(data->hours) & PCF85263_HOURS12_MASK;
    if(data->ampm == 1) {
        tmp[1] |= PCF85263_HOURS12_AMPM;
    }
#endif
    tmp[2]  = _dec2bcd(data->wday) & PCF85263_WEEKDAYS_MASK;
    
    /* Write data to the device */
    if(i2c_write_block(PCF85263_I2C_ADDRESS,PCF85263_RTC_MINUTE_ALARM2,tmp,3) != I2C_RET_OK) {
        /* Writing failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Read the alarm enables register value.
 * 
 * @param[in]   value       Alarm enables value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_stw_alarm_enables(uint8_t* value) {
    /* Get the alarm enables register (0x10) */
    return pcf85263_read_reg(PCF85263_STW_ALARM_ENABLES, value);
}


/*!
 * Write the alarm enables register value.
 * 
 * @param[in]   value       Alarm enables value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_stw_alarm_enables(uint8_t value) {
    /* Set the alarm enables registers (0x10) */
    return pcf85263_write_reg(PCF85263_STW_ALARM_ENABLES, value);
}


/*!
 * Read the alarm1 date-time registers (RTC mode).
 * 
 * @param[out]  data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_stw_alarm1(PCF85263_CNTTIME_t* data) {
    /* Data array to be filled */
    uint8_t tmp[5] = {0};
    /* Read raw data */
    if(i2c_read_block(PCF85263_I2C_ADDRESS,PCF85263_STW_SECOND_ALM1,tmp,5) != I2C_RET_OK) {
        /* Reading failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Extract information from raw data */
#if PCF85263_100TH_SECONDS_ENABLE==1
    data->msec10    = 0;                    /**< not available for alarm1 */
#endif
    data->seconds   = _bcd2dec(tmp[0] & PCF85263_SECONDS_MASK);
    data->minutes   = _bcd2dec(tmp[1] & PCF85263_MINUTES_MASK);
    data->hours     = _bcd2dec(tmp[2]);
    data->hours    += _bcd2dec(tmp[3]) * 100;
    data->hours    += _bcd2dec(tmp[4]) * 10000UL;
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Write the alarm1 date-time registers (RTC mode).
 * 
 * @param[in]   data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_stw_alarm1(PCF85263_CNTTIME_t* data) {
    /* Temporary data array */
    uint8_t tmp[5] = {0};
    /* Write information into data array */
    tmp[0]  = _dec2bcd(data->seconds) & PCF85263_SECONDS_MASK;
    tmp[1]  = _dec2bcd(data->minutes) & PCF85263_MINUTES_MASK;
    tmp[2]  = _dec2bcd(data->hours % 100);
    tmp[3]  = _dec2bcd((uint8_t)(data->hours / 100) % 100);
    tmp[4]  = _dec2bcd((uint8_t)(data->hours / 10000UL) % 100);
    
    /* Write data to the device */
    if(i2c_write_block(PCF85263_I2C_ADDRESS,PCF85263_STW_SECOND_ALM1,tmp,5) != I2C_RET_OK) {
        /* Writing failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Read the alarm2 date-time registers (RTC mode).
 * 
 * @param[out]  data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_stw_alarm2(PCF85263_CNTTIME_t* data) {
    /* Data array to be filled */
    uint8_t tmp[3] = {0};
    /* Read raw data */
    if(i2c_read_block(PCF85263_I2C_ADDRESS,PCF85263_STW_MINUTE_ALM2,tmp,3) != I2C_RET_OK) {
        /* Reading failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Extract information from raw data */
#if PCF85263_100TH_SECONDS_ENABLE==1
    data->msec10    = 0;                    /**< not available for alarm2 */
#endif
    data->seconds   = 0;                    /**< not available for alarm2 */
    data->minutes   = _bcd2dec(tmp[0] & PCF85263_MINUTES_MASK);
    data->hours     = _bcd2dec(tmp[1]);
    data->hours    += _bcd2dec(tmp[2]) * 100;
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Write the alarm2 date-time registers (RTC mode).
 * 
 * @param[in]   data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_stw_alarm2(PCF85263_CNTTIME_t* data) {
    /* Temporary data array */
    uint8_t tmp[3] = {0};
    /* Write information into data array */
    tmp[0]  = _dec2bcd(data->minutes) & PCF85263_MINUTES_MASK;
    tmp[1]  = _dec2bcd(data->hours % 100);
    tmp[2]  = _dec2bcd((uint8_t)(data->hours / 100) % 100);
    
    /* Write data to the device */
    if(i2c_write_block(PCF85263_I2C_ADDRESS,PCF85263_STW_MINUTE_ALM2,tmp,3) != I2C_RET_OK) {
        /* Writing failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Read the timestamp mode register value (RTC mode).
 * 
 * @param[in]   value       Timestamp mode value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_rtc_timestamp_mode(uint8_t* value) {
    /* Get the timestamp mode control register (0x23) */
    return pcf85263_read_reg(PCF85263_RTC_TSR_MODE, value);
}


/*!
 * Write the timestamp mode register value (RTC mode).
 * 
 * @param[in]   value       Timestamp mode value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_rtc_timestamp_mode(uint8_t value) {
    /* Set the timestamp mode control registers (0x23) */
    return pcf85263_write_reg(PCF85263_RTC_TSR_MODE, value);
}


/*!
 * Read the timestamp1 time and date (RTC mode).
 * 
 * @param[out]  data        Pointer to the date-time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_rtc_timestamp1(PCF85263_DATETIME_t* data) {
    /* Data array to be filled */
    uint8_t tmp[6] = {0};
    /* Read raw data */
    if(i2c_read_block(PCF85263_I2C_ADDRESS,PCF85263_RTC_TSR1_SECONDS,tmp,6) != I2C_RET_OK) {
        /* Reading failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Extract information from raw data */
#if PCF85263_100TH_SECONDS_ENABLE==1
    data->msec10    = 0;                    /**< not available for timestamps */
#endif
    data->seconds   = _bcd2dec(tmp[0] & PCF85263_SECONDS_MASK);
    data->minutes   = _bcd2dec(tmp[1] & PCF85263_MINUTES_MASK);
#if PCF85263_24H_MODE_ENABLE==1
    data->hours     = _bcd2dec(tmp[2] & PCF85263_HOURS24_MASK);
#else
    data->ampm      = (tmp[2] & PCF85263_HOURS12_AMPM) ? 1 : 0;
    data->hours     = _bcd2dec(tmp[2] & PCF85263_HOURS12_MASK);
#endif
    data->days      = _bcd2dec(tmp[3] & PCF85263_DAYS_MASK);
    data->wday      = 0;                    /**< not available for timestamps */
    data->months    = _bcd2dec(tmp[4] & PCF85263_MONTHS_MASK);
    data->years     = _bcd2dec(tmp[5]);
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Write the timestamp1 time and date (RTC mode).
 * 
 * @param[in]   data        Pointer to the date-time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_rtc_timestamp1(PCF85263_DATETIME_t* data) {
    /* Temporary data array */
    uint8_t tmp[6] = {0};
    /* Write information into data array */
    tmp[0]  = _dec2bcd(data->seconds) & PCF85263_SECONDS_MASK;
    tmp[1]  = _dec2bcd(data->minutes) & PCF85263_MINUTES_MASK;
#if PCF85263_24H_MODE_ENABLE==1
    tmp[2]  = _dec2bcd(data->hours) & PCF85263_HOURS24_MASK;
#else
    tmp[2]  = (_dec2bcd(data->hours) & PCF85263_HOURS12_MASK;
    if(data->ampm == 1) {
        tmp[2] |= PCF85263_HOURS12_AMPM;
    }
#endif
    tmp[3]  = _dec2bcd(data->days) & PCF85263_DAYS_MASK;
    tmp[4]  = _dec2bcd(data->months) & PCF85263_MONTHS_MASK;
    tmp[5]  = _dec2bcd(data->years);
    
    /* Write data to the device */
    if(i2c_write_block(PCF85263_I2C_ADDRESS,PCF85263_RTC_TSR1_SECONDS,tmp,6) != I2C_RET_OK) {
        /* Writing failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Read the timestamp2 time and date (RTC mode).
 * 
 * @param[out]  data        Pointer to the date-time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_rtc_timestamp2(PCF85263_DATETIME_t* data) {
    /* Data array to be filled */
    uint8_t tmp[6] = {0};
    /* Read raw data */
    if(i2c_read_block(PCF85263_I2C_ADDRESS,PCF85263_RTC_TSR2_SECONDS,tmp,6) != I2C_RET_OK) {
        /* Reading failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Extract information from raw data */
#if PCF85263_100TH_SECONDS_ENABLE==1
    data->msec10    = 0;                    /**< not available for timestamps */
#endif
    data->seconds   = _bcd2dec(tmp[0] & PCF85263_SECONDS_MASK);
    data->minutes   = _bcd2dec(tmp[1] & PCF85263_MINUTES_MASK);
#if PCF85263_24H_MODE_ENABLE==1
    data->hours     = _bcd2dec(tmp[2] & PCF85263_HOURS24_MASK);
#else
    data->ampm      = (tmp[2] & PCF85263_HOURS12_AMPM) ? 1 : 0;
    data->hours     = _bcd2dec(tmp[2] & PCF85263_HOURS12_MASK);
#endif
    data->days      = _bcd2dec(tmp[3] & PCF85263_DAYS_MASK);
    data->wday      = 0;                    /**< not available for timestamps */
    data->months    = _bcd2dec(tmp[4] & PCF85263_MONTHS_MASK);
    data->years     = _bcd2dec(tmp[5]);
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Write the timestamp2 time and date (RTC mode).
 * 
 * @param[in]   data        Pointer to the date-time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_rtc_timestamp2(PCF85263_DATETIME_t* data) {
    /* Temporary data array */
    uint8_t tmp[6] = {0};
    /* Write information into data array */
    tmp[0]  = _dec2bcd(data->seconds) & PCF85263_SECONDS_MASK;
    tmp[1]  = _dec2bcd(data->minutes) & PCF85263_MINUTES_MASK;
#if PCF85263_24H_MODE_ENABLE==1
    tmp[2]  = _dec2bcd(data->hours) & PCF85263_HOURS24_MASK;
#else
    tmp[2]  = (_dec2bcd(data->hours) & PCF85263_HOURS12_MASK;
    if(data->ampm == 1) {
        tmp[2] |= PCF85263_HOURS12_AMPM;
    }
#endif
    tmp[3]  = _dec2bcd(data->days) & PCF85263_DAYS_MASK;
    tmp[4]  = _dec2bcd(data->months) & PCF85263_MONTHS_MASK;
    tmp[5]  = _dec2bcd(data->years);
    
    /* Write data to the device */
    if(i2c_write_block(PCF85263_I2C_ADDRESS,PCF85263_RTC_TSR2_SECONDS,tmp,6) != I2C_RET_OK) {
        /* Writing failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Read the timestamp3 time and date (RTC mode).
 * 
 * @param[out]  data        Pointer to the date-time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_rtc_timestamp3(PCF85263_DATETIME_t* data) {
    /* Data array to be filled */
    uint8_t tmp[6] = {0};
    /* Read raw data */
    if(i2c_read_block(PCF85263_I2C_ADDRESS,PCF85263_RTC_TSR3_SECONDS,tmp,6) != I2C_RET_OK) {
        /* Reading failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Extract information from raw data */
#if PCF85263_100TH_SECONDS_ENABLE==1
    data->msec10    = 0;                    /**< not available for timestamps */
#endif
    data->seconds   = _bcd2dec(tmp[0] & PCF85263_SECONDS_MASK);
    data->minutes   = _bcd2dec(tmp[1] & PCF85263_MINUTES_MASK);
#if PCF85263_24H_MODE_ENABLE==1
    data->hours     = _bcd2dec(tmp[2] & PCF85263_HOURS24_MASK);
#else
    data->ampm      = (tmp[2] & PCF85263_HOURS12_AMPM) ? 1 : 0;
    data->hours     = _bcd2dec(tmp[2] & PCF85263_HOURS12_MASK);
#endif
    data->days      = _bcd2dec(tmp[3] & PCF85263_DAYS_MASK);
    data->wday      = 0;                    /**< not available for timestamps */
    data->months    = _bcd2dec(tmp[4] & PCF85263_MONTHS_MASK);
    data->years     = _bcd2dec(tmp[5]);
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Write the timestamp3 time and date (RTC mode).
 * 
 * @param[in]   data        Pointer to the date-time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_rtc_timestamp3(PCF85263_DATETIME_t* data) {
    /* Temporary data array */
    uint8_t tmp[6] = {0};
    /* Write information into data array */
    tmp[0]  = _dec2bcd(data->seconds) & PCF85263_SECONDS_MASK;
    tmp[1]  = _dec2bcd(data->minutes) & PCF85263_MINUTES_MASK;
#if PCF85263_24H_MODE_ENABLE==1
    tmp[2]  = _dec2bcd(data->hours) & PCF85263_HOURS24_MASK;
#else
    tmp[2]  = (_dec2bcd(data->hours) & PCF85263_HOURS12_MASK;
    if(data->ampm == 1) {
        tmp[2] |= PCF85263_HOURS12_AMPM;
    }
#endif
    tmp[3]  = _dec2bcd(data->days) & PCF85263_DAYS_MASK;
    tmp[4]  = _dec2bcd(data->months) & PCF85263_MONTHS_MASK;
    tmp[5]  = _dec2bcd(data->years);
    
    /* Write data to the device */
    if(i2c_write_block(PCF85263_I2C_ADDRESS,PCF85263_RTC_TSR3_SECONDS,tmp,6) != I2C_RET_OK) {
        /* Writing failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Read the timestamp mode register value (stopwatch mode).
 * 
 * @param[in]   value       Timestamp mode value read
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_stw_timestamp_mode(uint8_t* value) {
    /* Get the timestamp mode control register (0x23) */
    return pcf85263_read_reg(PCF85263_STW_TSR_MODE, value);
}


/*!
 * Write the timestamp mode register value (stopwatch mode).
 * 
 * @param[in]   value       Timestamp mode value to be written
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_stw_timestamp_mode(uint8_t value) {
    /* Set the timestamp mode control registers (0x23) */
    return pcf85263_write_reg(PCF85263_STW_TSR_MODE, value);
}


/*!
 * Read the timestamp1 time (stop-watch mode).
 * 
 * @param[out]  data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_stw_timestamp1(PCF85263_CNTTIME_t* data) {
    /* Data array to be filled */
    uint8_t tmp[5] = {0};
    /* Read raw data */
    if(i2c_read_block(PCF85263_I2C_ADDRESS,PCF85263_STW_TSR1_SECONDS,tmp,5) != I2C_RET_OK) {
        /* Reading failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Extract information from raw data */
#if PCF85263_100TH_SECONDS_ENABLE==1
    data->msec10    = 0;                    /**< not available for timestamps */
#endif
    data->seconds   = _bcd2dec(tmp[0] & PCF85263_SECONDS_MASK);
    data->minutes   = _bcd2dec(tmp[1] & PCF85263_MINUTES_MASK);
    data->hours     = _bcd2dec(tmp[2]);
    data->hours    += _bcd2dec(tmp[3]) * 100;
    data->hours    += _bcd2dec(tmp[4]) * 10000UL;
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Write the timestamp1 time (stop-watch mode).
 * 
 * @param[in]   data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_stw_timestamp1(PCF85263_CNTTIME_t* data) {
    /* Temporary data array */
    uint8_t tmp[5] = {0};
    /* Write information into data array */
    tmp[0]  = _dec2bcd(data->seconds) & PCF85263_SECONDS_MASK;
    tmp[1]  = _dec2bcd(data->minutes) & PCF85263_MINUTES_MASK;
    tmp[2]  = _dec2bcd(data->hours % 100);
    tmp[3]  = _dec2bcd((data->hours / 100) % 100);
    tmp[4]  = _dec2bcd((data->hours / 10000UL) % 100);
    
    /* Write data to the device */
    if(i2c_write_block(PCF85263_I2C_ADDRESS,PCF85263_STW_TSR1_SECONDS,tmp,5) != I2C_RET_OK) {
        /* Writing failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Read the timestamp2 time (stop-watch mode).
 * 
 * @param[out]  data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_stw_timestamp2(PCF85263_CNTTIME_t* data) {
    /* Data array to be filled */
    uint8_t tmp[5] = {0};
    /* Read raw data */
    if(i2c_read_block(PCF85263_I2C_ADDRESS,PCF85263_STW_TSR2_SECONDS,tmp,5) != I2C_RET_OK) {
        /* Reading failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Extract information from raw data */
#if PCF85263_100TH_SECONDS_ENABLE==1
    data->msec10    = 0;                    /**< not available for timestamps */
#endif
    data->seconds   = _bcd2dec(tmp[0] & PCF85263_SECONDS_MASK);
    data->minutes   = _bcd2dec(tmp[1] & PCF85263_MINUTES_MASK);
    data->hours     = _bcd2dec(tmp[2]);
    data->hours    += _bcd2dec(tmp[3]) * 100;
    data->hours    += _bcd2dec(tmp[4]) * 10000UL;
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Write the timestamp2 time (stop-watch mode).
 * 
 * @param[in]   data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_stw_timestamp2(PCF85263_CNTTIME_t* data) {
    /* Temporary data array */
    uint8_t tmp[5] = {0};
    /* Write information into data array */
    tmp[0]  = _dec2bcd(data->seconds) & PCF85263_SECONDS_MASK;
    tmp[1]  = _dec2bcd(data->minutes) & PCF85263_MINUTES_MASK;
    tmp[2]  = _dec2bcd(data->hours % 100);
    tmp[3]  = _dec2bcd((data->hours / 100) % 100);
    tmp[4]  = _dec2bcd((data->hours / 10000UL) % 100);
    
    /* Write data to the device */
    if(i2c_write_block(PCF85263_I2C_ADDRESS,PCF85263_STW_TSR2_SECONDS,tmp,5) != I2C_RET_OK) {
        /* Writing failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Read the timestamp3 time (stop-watch mode).
 * 
 * @param[out]  data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_get_stw_timestamp3(PCF85263_CNTTIME_t* data) {
    /* Data array to be filled */
    uint8_t tmp[5] = {0};
    /* Read raw data */
    if(i2c_read_block(PCF85263_I2C_ADDRESS,PCF85263_STW_TSR3_SECONDS,tmp,5) != I2C_RET_OK) {
        /* Reading failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Extract information from raw data */
#if PCF85263_100TH_SECONDS_ENABLE==1
    data->msec10    = 0;                    /**< not available for timestamps */
#endif
    data->seconds   = _bcd2dec(tmp[0] & PCF85263_SECONDS_MASK);
    data->minutes   = _bcd2dec(tmp[1] & PCF85263_MINUTES_MASK);
    data->hours     = _bcd2dec(tmp[2]);
    data->hours    += _bcd2dec(tmp[3]) * 100;
    data->hours    += _bcd2dec(tmp[4]) * 10000UL;
    
    /* Return with success */
    return PCF85263_RET_OK;
}


/*!
 * Write the timestamp3 time (stop-watch mode).
 * 
 * @param[in]   data        Pointer to the time data structure
 * @return      OK in case of success; ERROR otherwise
 */
PCF85263_RET_t pcf85263_set_stw_timestamp3(PCF85263_CNTTIME_t* data) {
    /* Temporary data array */
    uint8_t tmp[5] = {0};
    /* Write information into data array */
    tmp[0]  = _dec2bcd(data->seconds) & PCF85263_SECONDS_MASK;
    tmp[1]  = _dec2bcd(data->minutes) & PCF85263_MINUTES_MASK;
    tmp[2]  = _dec2bcd(data->hours % 100);
    tmp[3]  = _dec2bcd((data->hours / 100) % 100);
    tmp[4]  = _dec2bcd((data->hours / 10000UL) % 100);
    
    /* Write data to the device */
    if(i2c_write_block(PCF85263_I2C_ADDRESS,PCF85263_STW_TSR3_SECONDS,tmp,5) != I2C_RET_OK) {
        /* Writing failed */
        return PCF85263_RET_ERROR;
    }
    
    /* Return with success */
    return PCF85263_RET_OK;
}
