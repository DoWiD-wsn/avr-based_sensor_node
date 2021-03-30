/**
 *  Source file for Circular Buffer functionality.
 */

/***** INCLUDES ***************************************************************/
/* STD */
#include <stddef.h>
/* OWN */
#include "cbuffer.h"


/***** MACROS *****************************************************************/


/***** ENUMERATION ************************************************************/


/***** STRUCTURES *************************************************************/


/***** GLOBAL VARIABLES *******************************************************/


/***** LOCAL FUNCTION PROTOTYPES **********************************************/


/***** FUNCTIONS **************************************************************/
/*
 * Initialize the ring buffer structure
 */
int8_t cbuf_init(cbuf_t* cb) {
    uint8_t i;
    /* Check if the pointer is valid */
    if(cb != NULL) {
        /* Reset buffer */
        for(i=0; i<CBUF_SIZE; i++) {
            /* Clear data element */
            cb->data[i] = 0;
        }
        /* Reset indices */
        cb->wr_i    = 0;
        cb->rd_i    = 0;
        /* Reset cnt & status */
        cb->cnt     = 0;
        cb->stat    = CBUF_STAT_EMPTY;
        /* return success */
        return CBUF_RET_OK;
    }
    return CBUF_RET_ERROR;
}


/*
 * Flush the ring buffer structure
 */
int8_t cbuf_flush(cbuf_t* cb) {
    /* Flushing does the same as init */
    return cbuf_init(cb);
}


/*
 * Push an element on the ring buffer
 */
int8_t cbuf_push(cbuf_t* cb, uint8_t byte) {
    /* Check if the pointer is valid */
    if(cb != NULL) {
        /* Check if buffer is full */
        if(cbuf_isfull(cb)) {
            /* Buffer is full, cannot push */
            return CBUF_RET_FULL;
        }
        /* Put one element */
        cb->data[cb->wr_i] = byte;
        /* Increment write index */
        cb->wr_i = (cb->wr_i+1) % CBUF_SIZE;
        /* Increment element counter */
        cb->cnt++;
        /* Check if buffer is full (equal read and write index) */
        if(cb->wr_i == cb->rd_i) {
            /* Set buffer full flag */
            cb->stat = CBUF_STAT_FULL;
            /* return success */
            return CBUF_RET_OK;
        }
        /* Check if buffer was marked empty */
        if (cb->stat == CBUF_STAT_EMPTY) {
            cb->stat = CBUF_STAT_NORMAL;
        }
        /* return success */
        return CBUF_RET_OK;
    }
    /* return error */
    return CBUF_RET_ERROR;
}


/*
 * Pop an element from the ring buffer
 */
int8_t cbuf_pop(cbuf_t* cb, uint8_t* byte) {
    /* Check if the pointer is valid */
    if(cb != NULL) {
        /* Check if buffer is empty */
        if(cbuf_isempty(cb)) {
            /* Buffer is empty, nothing to pop */
            return CBUF_RET_EMPTY;
        }
        /* Get one element */
        *byte = cb->data[cb->rd_i];
        /* Increment read index */
        cb->rd_i = (cb->rd_i+1) % CBUF_SIZE;
        /* Decrement element counter */
        cb->cnt--;
        /* Check if buffer is empty (equal read and write index) */
        if(cb->rd_i == cb->wr_i) {
            /* Set buffer empty flag */
            cb->stat = CBUF_STAT_EMPTY;
            /* return success */
            return CBUF_RET_OK;
        }
        /* Check if buffer was marked full */
        if (cb->stat == CBUF_STAT_FULL) {
            cb->stat = CBUF_STAT_NORMAL;
        }
        /* return success */
        return CBUF_RET_OK;
    }
    /* return error */
    return CBUF_RET_ERROR;
}


/*
 * Get the number of stored elements (cnt)
 */
int8_t cbuf_getcnt(cbuf_t* cb) {
    /* Check if the pointer is valid */
    if(cb != NULL) {
        /* Check if buffer is empty */
        return cb->cnt;
    }
    /* return error */
    return CBUF_RET_ERROR;
}


/*
 * Check if the ring buffer is empty
 */
int8_t cbuf_isempty(cbuf_t* cb) {
    /* Check if the pointer is valid */
    if(cb != NULL) {
        /* Check if buffer is empty */
        if(cb->stat == CBUF_STAT_EMPTY) {
            /* Buffer is empty */
            return 1;
        } else {
            /* Buffer not empty */
            return 0;
        }
    }
    /* return error */
    return CBUF_RET_ERROR;
}


/*
 * Check if the ring buffer is full
 */
int8_t cbuf_isfull(cbuf_t* cb) {
    /* Check if the pointer is valid */
    if(cb != NULL) {
        /* Check if buffer is full */
        if(cb->stat == CBUF_STAT_FULL) {
            /* Buffer is full */
            return 1;
        } else {
            /* Buffer not full */
            return 0;
        }
    }
    /* return error */
    return CBUF_RET_ERROR;
}
