/**
 *  Source file for SENOR MESSAGING functionality.
 */

/***** INCLUDES ***************************************************************/
#include "sensor_msg.h"


/***** FUNCTIONS **************************************************************/
/*
 *  Fill the message structure (take care of byte order!)
 */
void sen_msg_fill(sen_msg_t* msg, uint32_t time, uint8_t type, float value, uint8_t sreg) {
#if SEN_MSG_REVERSE_BYTE_ORDER
    msg->struc.time     = SEN_MSG_SWAP_32BIT(time);
    msg->struc.type     = type;
    msg->struc.value    = value;
    msg->struc.sreg     = sreg;
#else
    msg->struc.time     = time;
    msg->struc.type     = type;
    msg->struc.value    = value;
    msg->struc.sreg     = sreg;
#endif
}


/*
 *  Read from the message structure (take care of byte order!)
 */
void sen_msg_read(sen_msg_t* msg, uint32_t* time, uint8_t* type, float* value, uint8_t* sreg) {
#if SEN_MSG_REVERSE_BYTE_ORDER
    *time       = SEN_MSG_SWAP_32BIT(msg->struc.time);
    *type       = msg->struc.type;
    *value      = msg->struc.value;
    *sreg       = msg->struc.sreg;
#else
    *time       = msg->struc.time;
    *type       = msg->struc.type;
    *value      = msg->struc.value;
    *sreg       = msg->struc.sreg;
#endif
}
