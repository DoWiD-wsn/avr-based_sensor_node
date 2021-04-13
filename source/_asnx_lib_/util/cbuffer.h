/*****
 * @brief   ASN(x) circular buffer library
 *
 * Library to enable circular buffer functionality.
 *
 * @file    /_asnx_lib_/util/cbufffer.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/13 $
 *****/

#ifndef _ASNX_CBUF_H_
#define _ASNX_CBUF_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>


/***** MACROS *********************************************************/
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


/***** STRUCTURES *****************************************************/
/***
 * A structure to represent a circular buffer.
 ***/
typedef struct {
    uint8_t data[CBUF_SIZE];    /**< Buffer for send or receive data */
    uint8_t wr_i;               /**< Write buffer index */
    uint8_t rd_i;               /**< Read buffer index */
    uint16_t cnt;               /**< Number of items in the buffer */
    uint8_t stat;               /**< Buffer-status flag (normal/empty/full) */
} cbuf_t;


/***** FUNCTION PROTOTYPES ********************************************/
int8_t cbuf_init(cbuf_t* cb);
int8_t cbuf_flush(cbuf_t* cb);
int8_t cbuf_push(cbuf_t* cb, uint8_t byte);
int8_t cbuf_pop(cbuf_t* cb, uint8_t* byte);
int8_t cbuf_getcnt(cbuf_t* cb);
int8_t cbuf_isempty(cbuf_t* cb);
int8_t cbuf_isfull(cbuf_t* cb);


#endif // _ASNX_CBUF_H_
