/**
 *  Header file for AVR FUSES functionality.
 */

#ifndef _AVR_FUSES_H_
#define _AVR_FUSES_H_

/***** INCLUDES ***************************************************************/
#include <stdint.h>


/***** MACROS *****************************************************************/
#ifndef F_CPU
# warning "F_CPU not defined for \"fuses.h\""
# define F_CPU 16000000UL
#endif


/***** ENUMERATION ************************************************************/
/* Enumeration type for the low fuse byte */
typedef enum {
        FUSES_LOW_CKSEL0    = 0,    /* Select Clock Source */
        FUSES_LOW_CKSEL1    = 1,    /* Select Clock Source */
        FUSES_LOW_CKSEL2    = 2,    /* Select Clock Source */
        FUSES_LOW_CKSEL3    = 3,    /* Select Clock Source */
        FUSES_LOW_SUT0      = 4,    /* Select start-up time */
        FUSES_LOW_SUT1      = 5,    /* Select start-up time */
        FUSES_LOW_CKOUT     = 6,    /* Clock output */
        FUSES_LOW_CKDIV8    = 7     /* Divide clock by 8 */
    } fuses_low_t;


/* Enumeration type for the high fuse byte */
typedef enum {
        FUSES_HIGH_BOOTRST  = 0,
        FUSES_HIGH_BOOTSZ0  = 1,
        FUSES_HIGH_BOOTSZ1  = 2,
        FUSES_HIGH_EESAVE   = 3,    /* EEPROM memory is preserved through chip erase */
        FUSES_HIGH_WDTON    = 4,    /* Watchdog Timer Always On */
        FUSES_HIGH_SPIEN    = 5,    /* Enable Serial programming and Data Downloading */
        FUSES_HIGH_DWEN     = 6,    /* debugWIRE Enable */
        FUSES_HIGH_RSTDISBL = 7     /* External reset disable */
    } fuses_high_t;


/* Enumeration type for the extended fuse byte */
typedef enum {
        FUSES_EXT_BODLEVEL0 = 0,    /* Brown-out Detector trigger level */
        FUSES_EXT_BODLEVEL1 = 1,    /* Brown-out Detector trigger level */
        FUSES_EXT_BODLEVEL2 = 2     /* Brown-out Detector trigger level */
    } fuses_ext_t;


/* Enumeration type for state of a fuse */
typedef enum {
        FUSES_PROGRAMMED = 0,
        FUSES_UNPROGRAMMED = 1
    } fuses_state_t;


/***** FUNCTION PROTOTYPES ****************************************************/
/* Get the fuse bytes */
uint8_t fuses_get_low(void);
uint8_t fuses_get_high(void);
uint8_t fuses_get_ext(void);
/* Check if a certain fuse is programmed */
fuses_state_t fuses_is_low_set(fuses_low_t fuse);
fuses_state_t fuses_is_high_set(fuses_high_t fuse);
fuses_state_t fuses_is_ext_set(fuses_ext_t fuse);
/* Check clock source */
uint32_t fuses_get_fcpu(void);


#endif // _AVR_FUSES_H_
