/*!
 * @brief   ASN(x) circular buffer library -- header file
 *
 * Library to enable circular buffer functionality.
 *
 * @file    /_asnx_lib_/util/cbuffer.h
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 */

#ifndef _ASNX_CBUF_H_
#define _ASNX_CBUF_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
#include <stddef.h>


/***** MACROS *********************************************************/
/*! Ring buffer size [byte] */
#define CBUF_SIZE                   (200)
/* Buffer status */
#define CBUF_STAT_NORMAL            (0)     /**< Buffer status normal */
#define CBUF_STAT_EMPTY             (1)     /**< Buffer status empty */
#define CBUF_STAT_FULL              (2)     /**< Buffer status full */
/* Function return values */
#define CBUF_RET_OK                 (0)
#define CBUF_RET_ERROR              (-1)
#define CBUF_RET_EMPTY              (-2)
#define CBUF_RET_FULL               (-3)


/***** STRUCTURES *****************************************************/
/*!
 * A structure to represent a circular buffer.
 */
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
