/**
 *  Source file for AVR FUSES functionality.
 */

/***** INCLUDES ***************************************************************/
#include "fuses.h"
#include <avr/io.h>
#include <avr/boot.h>


/***** FUNCTIONS **************************************************************/

/*** Get the fuse bytes ***/

/*
 * Get the value of the low fuse byte
 */
uint8_t fuses_get_low(void) {
    return boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
}


/*
 * Get the value of the high fuse byte
 */
uint8_t fuses_get_high(void) {
    return boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
}


/*
 * Get the value of the extended fuse byte
 */
uint8_t fuses_get_ext(void) {
    return boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
}


/*** Check if a certain fuse is programmed ***/

/*
 * Check if a certain fuse of the low byte is set
 */
fuses_state_t fuses_is_low_set(fuses_low_t fuse) {
    if(fuses_get_low() & _BV(fuse)) {
        return FUSES_UNPROGRAMMED;
    } else {
        return FUSES_PROGRAMMED;
    }
}


/*
 * Check if a certain fuse of the high byte is set
 */
fuses_state_t fuses_is_high_set(fuses_high_t fuse) {
    if(fuses_get_high() & _BV(fuse)) {
        return FUSES_UNPROGRAMMED;
    } else {
        return FUSES_PROGRAMMED;
    }
}

/*
 * Check if a certain fuse of the extended byte is set
 */
fuses_state_t fuses_is_ext_set(fuses_ext_t fuse) {
    if(fuses_get_ext() & _BV(fuse)) {
        return FUSES_UNPROGRAMMED;
    } else {
        return FUSES_PROGRAMMED;
    }
}


/*** Check clock source ***/

/*
 * Check the CKSEL and CKDIV8 fuses and return the actual F_CPU
 */
uint32_t fuses_get_fcpu(void) {
    /* Get the clock source selection */
    uint8_t cksel = (fuses_get_low() & 0x0F);
    /* Check which clock source is selected */
    switch(cksel) {
        /* Internal RC oscillator */
        case 0x02:
            /* Check if CKDIV8 is programmed */
            if(fuses_is_low_set(FUSES_LOW_CKDIV8) == FUSES_UNPROGRAMMED) {
                /* F_CPU is the internal 8MHz */
                return 8000000UL;
            } else {
                /* F_CPU is the internal 8MHz divided by 8 = 1MHz */
                return 1000000UL;
            }
        /* Internal 128kHz RC oscillator */
        case 0x03:
            return 128000UL;
        /* Reserved - Ext. Clock - Crystal Oscillator */
        default:
            /* Assume F_CPU is properly set */
            if(fuses_is_low_set(FUSES_LOW_CKDIV8) == FUSES_UNPROGRAMMED) {
                /* F_CPU */
                return F_CPU;
            } else {
                /* F_CPUdivided by 8 = 1MHz */
                return F_CPU/8;
            }
    }
}
