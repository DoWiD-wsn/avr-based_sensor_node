/*****
 * @brief   ASN(x) sensor network message library
 *
 * Library for functionality regarding the messaging of sensor values.
 *
 * @file    /_asnx_lib_/util/sensor_msg.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/19 $
 *****/

/***** INCLUDES *******************************************************/
#include "sensor_msg.h"
/*** ASNX ***/
#include "util/fixed_point.h"


/***** FUNCTIONS ******************************************************/
/***
 * Fill the message structure.
 * 
 * @param[out]  msg     Pointer to the message structure to be filled
 * @param[in]   time    Time stamp (or counter) of the message
 * @param[in]   cnt     Number of measurements included
 * @param[in]   values  Pointer to the measurements field
 ***/
void sen_msg_fill(SEN_MSG_u* msg, uint16_t time, uint8_t cnt, SEN_VALUE_t* values) {
    /* Copy header information */
    msg->struc.time     = time;
    msg->struc.cnt      = cnt;
    /* Copy sensor measurements */
    uint8_t i;
    for(i=0; i<cnt; i++) {
        msg->struc.values[i].type = values[i].type;
        msg->struc.values[i].value = values[i].value;
    }
}


/***
 * Read from the message structure.
 * 
 * @param[in]   msg     Pointer to the message structure
 * @param[out]  time    Pointer to the time stamp to be filled
 * @param[out]  cnt     Pointer to the number of measurements to be filled
 * @param[out]  values  Pointer to the measurements field to be filled
 ***/
void sen_msg_read(SEN_MSG_u* msg, uint16_t* time, uint8_t* cnt, SEN_VALUE_t* values) {
    /* Copy header information */
    *time       = msg->struc.time;
    *cnt        = msg->struc.cnt;
    /* Copy sensor measurements */
    uint8_t i;
    for(i=0; i<*cnt; i++) {
        values[i].type = msg->struc.values[i].type;
        values[i].value = msg->struc.values[i].value;
    }
}
