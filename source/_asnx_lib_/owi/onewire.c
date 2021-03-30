/**
 *  Source file for OneWire functionality.
 */

/***** INCLUDES ***************************************************************/
#include "onewire.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>


/***** GLOBAL VARIABLES *******************************************************/
#if ONEWIRE_SEARCH
/* Global search state */
unsigned char ROM_NO[8];
uint8_t last_discrepancy;
uint8_t last_family_discrepancy;
uint8_t last_device_flag;
#endif


/***** LOCAL FUNCTION PROTOTYPES **********************************************/


/***** WRAPPER FUNCTIONS ******************************************************/


/***** FUNCTIONS **************************************************************/
/*
 * Setup the onewire data structure.
 * 
 * onewire_t* gpio      Pointer to the structure to be filled
 * uint8_t ddr          DDRx
 * uint8_t port         PORTx
 * uint8_t pin          PINx
 * uint8_t portpin      Portpin, e.g., PD2
 */
void onewire_get(onewire_t* gpio, volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin) {
    gpio->ddr = ddr;
    gpio->port = port;
    gpio->pin = pin;
    gpio->portpin = portpin;
}


/*
 * Perform the onewire reset function.
 * 
 * We will wait up to 250us for the bus to come high, if it doesn't then it
 * is broken or shorted and we return a 0;
 * 
 * AVR318: DetectPresence()
 * 
 * onewire_t* gpio      Pointer to the data structure
 * return               1 if device is accessible; 0 otherwise
 */
uint8_t onewire_reset(onewire_t* gpio) {
    uint8_t ret = 0xFF;
    uint8_t retries = ONEWIRE_TIMEOUT;

    /* Disable interrupts */
    cli();
    
    /* Release bus */
    ONEWIRE_GPIO_INPUT(gpio);
    /* Wait until the bus is high */
    do {
        /* Check if timeout has been reached */
        if(--retries == 0) return 0;
        /* Wait for some time */
        _delay_ms(2);
    } while(!ONEWIRE_GPIO_READ(gpio));
    /* Drive bus low */
    ONEWIRE_GPIO_LOW(gpio);
    ONEWIRE_GPIO_OUTPUT(gpio);
    /* Delay H */
    _delay_us(ONEWIRE_DELAY_H);
    /* Release bus */
    ONEWIRE_GPIO_INPUT(gpio);
    /* Delay I */
    _delay_us(ONEWIRE_DELAY_I);
    /* Only one delay does not work */
    _delay_us(ONEWIRE_DELAY_I);
    /* Read bus state */
    ret = ONEWIRE_GPIO_READ(gpio) ? 0 : 1;
    /* Delay J */
    _delay_us(ONEWIRE_DELAY_J);
    /* Restore interrupts */
    sei();
    /* Return result */
    return ret;
}


/*
 * Stop forcing power onto the bus.
 * 
 * Only needed if the 'power' flag to write is used.
 * 
 * onewire_t* gpio      Pointer to the data structure
 */
void onewire_depower(onewire_t* gpio) {
    /* Release bus */
    ONEWIRE_GPIO_INPUT(gpio);
}


/*
 * Write a bit on the onewire interface
 * 
 * AVR318: WriteBitx()
 * 
 * onewire_t* gpio      Pointer to the data structure
 * uint8_t value        value to be written (1|0)
 */
void onewire_write_bit(onewire_t* gpio, uint8_t value) {
    /* Disable interrupts */
    cli();
    /* Drive bus low */
    ONEWIRE_GPIO_LOW(gpio);
    ONEWIRE_GPIO_OUTPUT(gpio);
    /* Delay A */
    _delay_us(ONEWIRE_DELAY_A);
    /* When we want to write 1, we release the line */
    if(value) {
        /* Release bus */
        ONEWIRE_GPIO_INPUT(gpio);
    }
    /* Delay B */
    _delay_us(ONEWIRE_DELAY_B);
    /* Release bus */
    ONEWIRE_GPIO_INPUT(gpio);
    /* Restore interrupts */
    sei();
}


/*
 * Read a bit from the onewire interface
 * 
 * AVR318: ReadBit()
 * 
 * onewire_t* gpio      Pointer to the data structure
 * return               value read (1|0)
 */
uint8_t onewire_read_bit(onewire_t* gpio) {
    uint8_t ret = 0;

    /* Disable interrupts */
    cli();
    /* Drive bus low */
    ONEWIRE_GPIO_LOW(gpio);
    ONEWIRE_GPIO_OUTPUT(gpio);
    /* Delay A */
    _delay_us(ONEWIRE_DELAY_A);
    /* Release bus */
    ONEWIRE_GPIO_INPUT(gpio);
    /* Delay E */
    _delay_us(ONEWIRE_DELAY_E);
    /* Read bus state */
    ret = ONEWIRE_GPIO_READ(gpio) ? 1 : 0;
    /* Delay F */
    _delay_us(ONEWIRE_DELAY_F);
    /* Restore interrupts */
    sei();
    /* Return result */
    return ret;
}


/*
 * Write a byte on the onewire interface
 * 
 * onewire_t* gpio      Pointer to the data structure
 * uint8_t byte         byte to be written
 * uint8_t power        parasite power mode (1 on | 0 off)
 */
void onewire_write_byte(onewire_t* gpio, uint8_t byte, uint8_t power) {
    uint8_t offset;
    
    /* Write all bits of the byte via onewire */
    for(offset = 0; offset<8; offset++) {
        /* Check if corresponding bit is set and write it */
        onewire_write_bit(gpio,(byte & _BV(offset)) ? 1 : 0);
    }
    /* Check if parasite power mode is deactivated */
    if(!power) {
        /* Set portpin back to input */
        ONEWIRE_GPIO_INPUT(gpio);
        /* Set portpin port value to low */
        ONEWIRE_GPIO_LOW(gpio);
    }
}


/*
 * Write several bytes on the onewire interface
 * 
 * onewire_t* gpio      Pointer to the data structure
 * uint8_t *bytes       pointer to the byte array
 * uint16_t count       number of bytes to be written
 * uint8_t power        parasite power mode (1 on | 0 off)
 */
void onewire_write(onewire_t* gpio, const uint8_t *bytes, uint16_t count, uint8_t power) {
    uint16_t i;
    /* Write the given amount of bytes */
    for(i=0; i<count; i++) {
        /* Write current byte */
        onewire_write_byte(gpio, bytes[i],power);
    }
    /* Check if parasite power mode is deactivated */
    if (!power) {
        /* Set portpin back to input */
        ONEWIRE_GPIO_INPUT(gpio);
        /* Set portpin port value to low */
        ONEWIRE_GPIO_LOW(gpio);
    }
}


/*
 * Read a byte from the onewire interface
 * 
 * onewire_t* gpio      Pointer to the data structure
 * return               byte read
 */
uint8_t onewire_read_byte(onewire_t* gpio) {
    uint8_t offset;
    uint8_t ret = 0;
    
    /* Write all bits of the byte via onewire */
    for (offset = 0; offset<8; offset++) {
        /* Check if read bit is set */
        if(onewire_read_bit(gpio)) {
            /* Set corresponding bit in return value */
            ret |= _BV(offset);
        }
    }
    /* Return the received byte */
    return ret;
}


/*
 * Read several byte from the onewire interface
 * 
 * onewire_t* gpio      Pointer to the data structure
 * uint8_t *bytes       pointer to the byte array
 * uint16_t count       number of bytes to be written
 */
void onewire_read(onewire_t* gpio, uint8_t *bytes, uint16_t count) {
    uint16_t i;
    /* Read the given amount of bytes */
    for(i=0; i<count; i++) {
        /* Read current byte */
        bytes[i] = onewire_read_byte(gpio);
    }
}


/*
 * Issue a onewire ROM select command
 * 
 * onewire_t* gpio      Pointer to the data structure
 * uint8_t rom          pointer to the rom array
 */
void onewire_select(onewire_t* gpio, const uint8_t rom[8]) {
    uint8_t i;
    /* Write the "select" ROM address */
    onewire_write_byte(gpio, ONEWIRE_ROM_SELECT, ONEWIRE_PARASITE_ON);
    /* Write the eight ROM bytes */
    for(i=0; i<8; i++) {
        /* Write current byte */
        onewire_write_byte(gpio, rom[i], ONEWIRE_PARASITE_ON);
    }
}


/*
 * Issue a onewire ROM skip command
 * 
 * onewire_t* gpio      Pointer to the data structure
 */
void onewire_skip(onewire_t* gpio) {
    /* Write the "select" ROM address */
    onewire_write_byte(gpio, ONEWIRE_ROM_SKIP, ONEWIRE_PARASITE_ON);
}



/******************************************/
/***** ***** ***** SEARCH ***** ***** *****/
/******************************************/
#if ONEWIRE_SEARCH
/*
 * Clear the search state so that if will start from the beginning again.
 */
void onewire_search_reset(void) {
    uint8_t i;
    /* Reset the search state */
    last_discrepancy = 0;
    last_family_discrepancy = 0;
    last_device_flag = ONEWIRE_SEARCH_NOT_FOUND;
    /* Clear ROM table */
    for(i=0; i<8; i++) {
        /* Clear entry */
        ROM_NO[i] = 0;
    }
}


/*
 * Search to find the device type 'family_code' on the next call to
 * onewire_search(*addr) if it is present.
 * 
 * uint8_t family_code  Device family code to be found
 */
void onewire_search_target(uint8_t family_code) {
    uint8_t i;
    /* Set the search state to find devices with the given family code */
    ROM_NO[0] = family_code;
    /* Clear the other entries */
    for(i=1; i<8; i++) {
        /* Clear entry */
        ROM_NO[i] = 0;
    }
    last_discrepancy = 64;
    last_family_discrepancy = 0;
    last_device_flag = ONEWIRE_SEARCH_NOT_FOUND;
}


/*
 * Perform a search.
 * 
 * onewire_t* gpio      Pointer to the data structure
 * uint8_t *addr        Address of a matching device (if found)
 * uint8_t family_code  Device family code to be found
 * 
 * return ONEWIRE_SEARCH_FOUND     -> device found, ROM number in ROM_NO buffer
 *        ONEWIRE_SEARCH_NOT_FOUND -> device not found, end of search
 */
uint8_t onewire_search(onewire_t* gpio, uint8_t *addr) {
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
    if (!last_device_flag) {
        /* Perform a onewire reset */
        if(!onewire_reset(gpio)) {
            /* Reset the search */
            last_discrepancy = 0;
            last_family_discrepancy = 0;
            last_device_flag = ONEWIRE_SEARCH_NOT_FOUND;
            /* Return no device found */
            return ONEWIRE_SEARCH_NOT_FOUND;
        }

        /* Write the "search" ROM address */
        onewire_write_byte(gpio, ONEWIRE_ROM_SEARCH, ONEWIRE_PARASITE_ON);

        /* Loop over the search space */
        do {
            /* Read a bit and its complement */
            id_bit = onewire_read_bit(gpio);
            cmp_id_bit = onewire_read_bit(gpio);

            /* Check for no devices on onewire */
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
                    if(id_bit_number < last_discrepancy) {
                        /* next then pick the same as last time */
                        search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
                    } else {
                        /* If equal to last pick 1, if not then pick 0 */
                        search_direction = (id_bit_number == last_discrepancy);
                    }

                    /* If 0 was picked then record its position in last_zero */
                    if(search_direction == 0) {
                        /* Get last zero */
                        last_zero = id_bit_number;
                        /* Check for last discrepancy in family */
                        if(last_zero < 9) {
                            /* Get last discrepancy in family */
                            last_family_discrepancy = last_zero;
                        }
                    }
                }

                /* Depending on the search direction ... */
                if (search_direction == 1) {
                    /* ... set the bit in the ROM byte */
                    ROM_NO[rom_byte_number] |= rom_byte_mask;
                } else {
                    /* ... clear the bit in the ROM byte */
                    ROM_NO[rom_byte_number] &= ~rom_byte_mask;
                }

                /* Serial number search direction write bit */
                onewire_write_bit(gpio, search_direction);

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
            last_discrepancy = last_zero;
            /* ... and check for last device */
            if(last_discrepancy == 0) {
                last_device_flag = ONEWIRE_SEARCH_FOUND;
            }
            search_result = ONEWIRE_SEARCH_FOUND;
        }
    }

    /* If no device found ... */
    if(!search_result || !ROM_NO[0]) {
        /* ... then reset counters so next 'search' will be like a first */
        last_discrepancy = 0;
        last_family_discrepancy = 0;
        last_device_flag = ONEWIRE_SEARCH_NOT_FOUND;
        search_result = ONEWIRE_SEARCH_NOT_FOUND;
    }
    /* Copy the found addresses */
    for(i=0; i<8; i++) {
        addr[i] = ROM_NO[i];
    }
    
    /* Return the search result */
    return search_result;
}
#endif



/******************************************/
/***** ***** *****  CRC   ***** ***** *****/
/******************************************/
#if ONEWIRE_CRC
/* The onewire CRC scheme is described in Maxim Application Note 27:
 * "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products" */

#if ONEWIRE_CRC8_TABLE
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


/*
 * Get the 8-bit CRC from the lookup table
 * 
 * uint8_t *addr        Device family code to be found
 * uint8_t len          Device family code to be found
 * 
 * return               8-bit CRC
 */
uint8_t onewire_crc8(const uint8_t *addr, uint8_t len) {
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
/*
 * Compute the 8-bit CRC
 * 
 * uint8_t *addr        Device family code to be found
 * uint8_t len          Device family code to be found
 * 
 * return               8-bit CRC
 */
uint8_t onewire_crc8(const uint8_t *addr, uint8_t len) {
    uint16_t i;
    uint8_t j;
    /* Intermediate CRC value */
    uint8_t crc = 0;
    /* Go over all bytes */
    for(i=0; i<len ; i++) {
        /* Compute CRC for the current byte */
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


#if ONEWIRE_CRC16
/*
 * Compute the 16-bit CRC
 * 
 * uint8_t* input       Array of bytes to checksum.
 * uint8_t len          How many bytes to use.
 * uint8_t crc          The crc starting value (optional)
 * 
 * return               1 if successful, 0 otherwise
 */
uint16_t onewire_crc16(const uint8_t* input, uint16_t len, uint16_t crc) {
    uint16_t i;
    /* Odd parity bits lookup table */
    static const uint8_t oddparity[16] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};
    /* Go over all bytes */
    for(i=0; i<len ; i++) {
        /* Even though we're just copying a byte from the input, we'll
         * be doing 16-bit computation with it. */
        uint16_t cdata = input[i];
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


/*
 * Check the 16-bit CRC
 * 
 * uint8_t* input       Array of bytes to checksum.
 * uint8_t len          How many bytes to use.
 * uint8_t* crc_i       The two CRC16 bytes in the received data.
 *                      This should just point into the received data, not at a 16-bit integer.
 * uint8_t crc          The crc starting value (optional)
 * 
 * return               1 if successful, 0 otherwise
 */
uint8_t onewire_crc16_check(const uint8_t* input, uint16_t len, const uint8_t* crc_i, uint16_t crc) {
    /* Get the inverted 16-bit CRC */
    crc = ~onewire_crc16(input,len,crc);
    /* Check if the CRC matches */
    return (((crc&0xFF) == crc_i[0]) && ((crc>>8) == crc_i[1]));
}
#endif
#endif

