/*****
 * @brief   ASN(x) one-wire interface (OWI) library
 *
 * Library to support one-wire interface (OWI) modules.
 * Globally deactivates interrupts during time-critical sections.
 *
 * @file    /_asnx_lib_/owi/owi.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1.0 $
 * @date    $Date: 2021/05/10 $
 * @see     https://github.com/szszoke/atmega328p/blob/master/onewire/
 * @see     https://hacksterio.s3.amazonaws.com/uploads/attachments/229743/OneWire.zip
 *****/


/***** INCLUDES *******************************************************/
#include "owi.h"
/*** AVR ***/
#include <avr/interrupt.h>
#if OWI_CRC
#  if OWI_CRC8_TABLE
#     include <avr/pgmspace.h>
#  endif
#endif
#include <util/delay.h>


/***** FUNCTIONS ******************************************************/
/***
 * Get a HW structure with the register values of a given GPIO.
 *
 * @param[out]  gpio    Pointer to the structure to be filled
 * @param[in]   ddr     Pointer to the GPIO's DDRx register
 * @param[in]   port    Pointer to the GPIO's PORTx register
 * @param[in]   pin     Pointer to the GPIO's PINx register
 * @param[in]   portpin Index of the GPIO pin
 ***/
void owi_get(hw_io_t* gpio, OWI_DATA_t* data, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin) {
    /* Call the respective HW function */
    hw_get_io(gpio, ddr, port, pin, portpin);
    /* Initialize the data structure */
    data->last_discrepancy = 0;
    data->last_family_discrepancy = 0;
    data->last_device_flag = 0;
    uint8_t i;
    for(i=0; i<OWI_ROM_SIZE; i++) {
        data->ROM_NO[i] = 0;
    }
}


/***
 * Perform a OWI reset.
 * Wait up to 250us for the bus to come high, if it doesn't, then it
 * is broken or shorted and we return a 0;
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 * @return      State of the OWI GPIO pin after reset
 ***/
uint8_t owi_reset(hw_io_t* gpio) {
    uint8_t ret = 0xFF;
    uint8_t retries = OWI_TIMEOUT;

    /* Time critical section start - disable interrupts */
    cli();
    
    /* Release bus */
    HW_GPIO_INPUT(gpio);
    /* Wait until the bus is high */
    do {
        /* Check if timeout has been reached */
        if(--retries == 0) {
            return 0;
        }
        /* Wait for some time */
        _delay_ms(2);
    } while(!HW_GPIO_READ(gpio));
    
    /* Drive bus low */
    HW_GPIO_LOW(gpio);
    HW_GPIO_OUTPUT(gpio);
    /* Delay H */
    _delay_us(OWI_DELAY_H);
    
    /* Release bus */
    HW_GPIO_INPUT(gpio);
    /* Delay I */
    _delay_us(OWI_DELAY_I);
    
    /* Read bus state (inverted) */
    ret = HW_GPIO_READ(gpio) ? 0 : 1;
    /* Delay J */
    _delay_us(OWI_DELAY_J);
    
    /* Time critical section end - restore interrupts */
    sei();
    
    /* Return result */
    return ret;
}


/***
 * Stop forcing power onto the OWI line.
 * Only needed if the 'power' flag to write is used.
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 ***/
void owi_depower(hw_io_t* gpio) {
    /* Release bus */
    HW_GPIO_INPUT(gpio);
}


/***
 * Write a bit on the OWI line.
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 * @param[in]   value   Bit value to be written (0 or 1)
 ***/
void owi_write_bit(hw_io_t* gpio, uint8_t value) {
    /* Disable interrupts */
    cli();
    
    /* Drive bus low */
    HW_GPIO_LOW(gpio);
    HW_GPIO_OUTPUT(gpio);
    /* Delay A */
    _delay_us(OWI_DELAY_A);
    /* When we want to write 1, we release the line */
    if(value) {
        /* Release bus */
        HW_GPIO_INPUT(gpio);
    }
    /* Delay B */
    _delay_us(OWI_DELAY_B);
    /* Release bus */
    HW_GPIO_INPUT(gpio);
    
    /* Restore interrupts */
    sei();
}


/***
 * Read a bit from the OWI line.
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 * @return      Bit value read from the line
 ***/
uint8_t owi_read_bit(hw_io_t* gpio) {
    uint8_t ret = 0;

    /* Disable interrupts */
    cli();
    
    /* Drive bus low */
    HW_GPIO_LOW(gpio);
    HW_GPIO_OUTPUT(gpio);
    /* Delay A */
    _delay_us(OWI_DELAY_A);
    /* Release bus */
    HW_GPIO_INPUT(gpio);
    /* Delay E */
    _delay_us(OWI_DELAY_E);
    /* Read bus state */
    ret = HW_GPIO_READ(gpio) ? 1 : 0;
    /* Delay F */
    _delay_us(OWI_DELAY_F);
    
    /* Restore interrupts */
    sei();
    
    /* Return result */
    return ret;
}


/***
 * Write a data byte on the OWI line.
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 * @param[in]   data    Data byte to be written (0 or 1)
 * @param[in]   power   Parasite power enable (0 or 1)
 ***/
void owi_write_byte(hw_io_t* gpio, uint8_t data, OWI_POWER_t power) {
    uint8_t offset;
    
    /* Write all bits of the byte via OWI */
    for(offset = 0; offset<8; offset++) {
        /* Check if corresponding bit is set and write it */
        owi_write_bit(gpio,(data & _BV(offset)) ? 1 : 0);
    }
    /* Check if parasite power mode is deactivated */
    if(!power) {
        /* Set portpin back to input */
        HW_GPIO_INPUT(gpio);
        /* Set portpin port value to low */
        HW_GPIO_LOW(gpio);
    }
}


/***
 * Write several data bytes on the OWI line.
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 * @param[in]   data    Data byte to be written (0 or 1)
 * @param[in]   len     Number of data bytes to be written
 * @param[in]   power   Parasite power enable (0 or 1)
 ***/
void owi_write(hw_io_t* gpio, const uint8_t *data, uint16_t len, OWI_POWER_t power) {
    uint16_t i;
    /* Write the given amount of bytes */
    for(i=0; i<len; i++) {
        /* Write current byte */
        owi_write_byte(gpio, data[i],power);
    }
    /* Check if parasite power mode is deactivated */
    if (!power) {
        /* Set portpin back to input */
        HW_GPIO_INPUT(gpio);
        /* Set portpin port value to low */
        HW_GPIO_LOW(gpio);
    }
}


/***
 * Read a data byte from the OWI line.
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 * @return      Data byte value read from the line
 ***/
uint8_t owi_read_byte(hw_io_t* gpio) {
    uint8_t offset;
    uint8_t ret = 0;
    
    /* Write all bits of the byte via OWI */
    for (offset = 0; offset<8; offset++) {
        /* Check if read bit is set */
        if(owi_read_bit(gpio)) {
            /* Set corresponding bit in return value */
            ret |= _BV(offset);
        }
    }
    /* Return the received byte */
    return ret;
}


/***
 * Read several data bytes from the OWI line.
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 * @param[out]  data    Data byte memory location
 * @param[in]   len     Number of data bytes to be read
 ***/
void owi_read(hw_io_t* gpio, uint8_t *data, uint16_t len) {
    uint16_t i;
    /* Read the given amount of bytes */
    for(i=0; i<len; i++) {
        /* Read current byte */
        data[i] = owi_read_byte(gpio);
    }
}


/***
 * Issue a OWI ROM select command.
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 * @param[in]   rom     ROM data bytes
 ***/
void owi_select(hw_io_t* gpio, uint8_t *addr) {
    uint8_t i;
    /* Write the "select" ROM address */
    owi_write_byte(gpio, OWI_ROM_SELECT, OWI_PARASITE_ON);
    /* Write the eight ROM bytes */
    for(i=0; i<OWI_ROM_SIZE; i++) {
        /* Write current byte */
        owi_write_byte(gpio, addr[i], OWI_PARASITE_ON);
    }
}


/***
 * Issue a OWI ROM skip command.
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 ***/
void owi_skip(hw_io_t* gpio) {
    /* Write the "skip" ROM address */
    owi_write_byte(gpio, OWI_ROM_SKIP, OWI_PARASITE_ON);
}


/***
 * Clear the search state so that it will start from the beginning again.
 ***/
void owi_search_reset(OWI_DATA_t* data) {
    uint8_t i;
    /* Reset the search state */
    data->last_discrepancy = 0;
    data->last_family_discrepancy = 0;
    data->last_device_flag = OWI_SEARCH_NOT_FOUND;
    /* Clear ROM table */
    for(i=0; i<OWI_ROM_SIZE; i++) {
        data->ROM_NO[i] = 0;
    }
}


/***
 * Search the device type 'family_code' on the next call to owi_search() if it is present.
 *
 * @param[in]   family_code     Device family code to be found
 ***/
void owi_search_target(OWI_DATA_t* data, uint8_t family_code) {
    uint8_t i;
    /* Set the search state to find devices with the given family code */
    data->ROM_NO[0] = family_code;
    /* Clear the other entries */
    for(i=1; i<OWI_ROM_SIZE; i++) {
        /* Clear entry */
        data->ROM_NO[i] = 0;
    }
    data->last_discrepancy = 64;
    data->last_family_discrepancy = 0;
    data->last_device_flag = OWI_SEARCH_NOT_FOUND;
}


/***
 * Perform a search for devices on the OWI line.
 *
 * @param[in]   gpio    Pointer to the GPIO structure
 * @param[out]  addr    Address of a matching device (if found)
 * @return      FOUND in case of success; NOT_FOUND otherwise
 ***/
OWI_SEARCH_t owi_search(hw_io_t* gpio, OWI_DATA_t* data, uint8_t *addr) {
    uint8_t i;
    /* Temporary and intermediate variables */
    uint8_t id_bit_number = 1;
    uint8_t last_zero = 0;
    uint8_t rom_byte_number = 0;
    uint8_t search_result = 0;
    uint8_t id_bit;
    uint8_t cmp_id_bit;
    uint8_t rom_byte_mask = 1;
    uint8_t search_direction;

    /* Check if the last call was not the last one */
    if (!data->last_device_flag) {
        /* Perform a OWI reset */
        if(!owi_reset(gpio)) {
            /* Reset the search */
            data->last_discrepancy = 0;
            data->last_family_discrepancy = 0;
            data->last_device_flag = OWI_SEARCH_NOT_FOUND;
            /* Return no device found */
            return OWI_SEARCH_NOT_FOUND;
        }
        /* Write the "search" ROM address */
        owi_write_byte(gpio, OWI_ROM_SEARCH, OWI_PARASITE_ON);

        /* Loop over the search space */
        do {
            /* Read a bit and its complement */
            id_bit = owi_read_bit(gpio);
            cmp_id_bit = owi_read_bit(gpio);

            /* Check for no devices on the OWI */
            if((id_bit == 1) && (cmp_id_bit == 1)) {
                /* Stop searching */
                break;
            } else {
                /* All devices coupled have 0 or 1 */
                if(id_bit != cmp_id_bit) {
                    /* Bit write value for search */
                    search_direction = id_bit;
                } else {
                    /* If this discrepancy is before the last discrepancy  on a previous ... */
                    if(id_bit_number < data->last_discrepancy) {
                        /* next then pick the same as last time */
                        search_direction = ((data->ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
                    } else {
                        /* If equal to last pick 1, if not then pick 0 */
                        search_direction = (id_bit_number == data->last_discrepancy);
                    }

                    /* If 0 was picked then record its position in last_zero */
                    if(search_direction == 0) {
                        /* Get last zero */
                        last_zero = id_bit_number;
                        /* Check for last discrepancy in family */
                        if(last_zero < 9) {
                            /* Get last discrepancy in family */
                            data->last_family_discrepancy = last_zero;
                        }
                    }
                }

                /* Depending on the search direction ... */
                if (search_direction == 1) {
                    /* ... set the bit in the ROM byte */
                    data->ROM_NO[rom_byte_number] |= rom_byte_mask;
                } else {
                    /* ... clear the bit in the ROM byte */
                    data->ROM_NO[rom_byte_number] &= ~rom_byte_mask;
                }

                /* Serial number search direction write bit */
                owi_write_bit(gpio, search_direction);

                /* Increment the byte counter  and shift the mask */
                id_bit_number++;
                rom_byte_mask <<= 1;

                /* If the mask is 0 ... */
                if(rom_byte_mask == 0) {
                    /* ... then go to new serial num byte and reset mask */
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        /* Loop through all ROM bytes 0-7 */
        }while(rom_byte_number<8);
        
        /* If the search was successful ... */
        if(!(id_bit_number < 65)) {
            /* ... then set the variables appropriately */
            data->last_discrepancy = last_zero;
            /* ... and check for last device */
            if(data->last_discrepancy == 0) {
                data->last_device_flag = OWI_SEARCH_FOUND;
            }
            search_result = OWI_SEARCH_FOUND;
        }
    }

    /* If no device found ... */
    if(!search_result || !data->ROM_NO[0]) {
        /* ... then reset counters so next 'search' will be like a first */
        data->last_discrepancy = 0;
        data->last_family_discrepancy = 0;
        data->last_device_flag = OWI_SEARCH_NOT_FOUND;
        search_result = OWI_SEARCH_NOT_FOUND;
    }
    
    /* Copy the found addresses */
    for(i=0; i<8; i++) {
        addr[i] = data->ROM_NO[i];
    }
    
    /* Return the search result */
    return search_result;
}


/* The OWI CRC scheme is described in Maxim Application Note 27:
 * "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products" */
#if OWI_CRC
#if OWI_CRC8_TABLE
/* This table comes from Dallas sample code where it is freely reusable,
 * though Copyright (C) 2000 Dallas Semiconductor Corporation */
static const uint8_t PROGMEM dscrc_table[] = {
      0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
    157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
     35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
    190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
     70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
    219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
    101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
    248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
    140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
     17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
    175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
     50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
    202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
     87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
    233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
    116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};


/***
 * Get the 8-bit CRC from the lookup table.
 *
 * @param[in]   addr    Device family code to be found
 * @param[in]   len     Memory space size
 * @return      8-bit CRC value from the LUT
 ***/
uint8_t owi_crc8(const uint8_t *addr, uint8_t len) {
    uint16_t i;
    /* Intermediate CRC value */
    uint8_t crc = 0;
    /* Go over all bytes */
    for(i=0; i<len ; i++) {
        /* Get CRC from the lookup table */
        crc = pgm_read_byte(dscrc_table + (crc ^ *addr++));
    }
    /* Return the calculated CRC */
    return crc;
}


#else
/***
 * Calculate the 8-bit CRC.
 *
 * @param[in]   addr    Device family code to be found
 * @param[in]   len     Memory space size
 * @return      Calculated 8-bit CRC value
 ***/
uint8_t owi_crc8(const uint8_t *addr, uint8_t len) {
    uint16_t i;
    uint8_t j;
    /* Intermediate CRC value */
    uint8_t crc = 0;
    /* Go over all bytes */
    for(i=0; i<len ; i++) {
        /* Calculate CRC for the current byte */
        uint8_t inbyte = *addr++;
        for(j=8; j>0; j--) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    /* Return the calculated CRC */
    return crc;
}
#endif


#if OWI_CRC16
/***
 * Calculate the 16-bit CRC.
 *
 * @param[in]   data    Array of bytes to checksum.
 * @param[in]   len     Number of data bytes
 * @param[in]   crc     The CRC starting value
 * @return      Calculated 16-bit CRC value
 ***/
uint16_t owi_crc16(const uint8_t* data, uint16_t len, uint16_t crc) {
    uint16_t i;
    /* Odd parity bits lookup table */
    static const uint8_t oddparity[16] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};
    /* Go over all bytes */
    for(i=0; i<len ; i++) {
        /* Even though we're just copying a byte from the input data, we'll
         * be doing 16-bit computation with it. */
        uint16_t cdata = data[i];
        cdata = (cdata ^ crc) & 0xff;
        crc >>= 8;

        if(oddparity[cdata & 0x0F]^oddparity[cdata >> 4]) {
            crc ^= 0xC001;
        }

        cdata <<= 6;
        crc ^= cdata;
        cdata <<= 1;
        crc ^= cdata;
    }
    /* Return the calculated CRC */
    return crc;
}


/***
 * Check a given 16-bit CRC.
 *
 * @param[in]   data    Array of bytes to checksum.
 * @param[in]   len     Number of data bytes
 * @param[in]   crc_i   The two CRC16 bytes in the received data.
 * @param[in]   crc     The CRC starting value
 * @return      1 if CRC matches; 0 otherwise
 ***/
uint8_t owi_crc16_check(const uint8_t* input, uint16_t len, const uint8_t* crc_i, uint16_t crc) {
    /* Get the inverted 16-bit CRC */
    crc = ~owi_crc16(input,len,crc);
    /* Check if the CRC matches */
    return (((crc&0xFF) == crc_i[0]) && ((crc>>8) == crc_i[1]));
}
#endif
#endif

