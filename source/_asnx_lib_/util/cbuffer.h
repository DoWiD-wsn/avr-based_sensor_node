/**
 *  Header file for Circular Buffer functionality.
 */

#ifndef _ASNX_CBUF_H_
#define _ASNX_CBUF_H_

/***** INCLUDES ***************************************************************/
#include <stdint.h>


/***** MACROS *****************************************************************/
/* Ring buffer size [byte] */
#define CBUF_SIZE                   (100)
/* Buffer status */
#define CBUF_STAT_NORMAL            (0)
#define CBUF_STAT_EMPTY             (1)
#define CBUF_STAT_FULL              (2)
/* Function return values */
#define CBUF_RET_OK                 (0)
#define CBUF_RET_ERROR              (-1)
#define CBUF_RET_EMPTY              (-2)
#define CBUF_RET_FULL               (-3)



/***** GLOBAL VARIABLES *******************************************************/


/***** ENUMERATION ************************************************************/


/***** STRUCTURES *************************************************************/
/* Circular buffer */
typedef struct {
    /* Buffer for send or receive data */
    uint8_t data[CBUF_SIZE];
    /* Read/Write buffer index */
    uint8_t wr_i;
    uint8_t rd_i;
    /* Number of items in the buffer */
    uint16_t cnt;
    /* Buffer-status flag (normal/empty/full) */
    uint8_t stat;
} cbuf_t;


/***** FUNCTION PROTOTYPES ****************************************************/
int8_t cbuf_init(cbuf_t* cb);
int8_t cbuf_flush(cbuf_t* cb);
int8_t cbuf_push(cbuf_t* cb, uint8_t byte);
int8_t cbuf_pop(cbuf_t* cb, uint8_t* byte);
int8_t cbuf_getcnt(cbuf_t* cb);
int8_t cbuf_isempty(cbuf_t* cb);
int8_t cbuf_isfull(cbuf_t* cb);


/***** INLINE FUNCTIONS *******************************************************/


#endif // _ASNX_CBUF_H_
