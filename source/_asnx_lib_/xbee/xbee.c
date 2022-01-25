/*!
 * @brief   ASN(x) Xbee 3 library -- source file
 *
 * Library to the Xbee 3 module accessible via UART.
 *
 * @file    /_asnx_lib_/xbee/xbee.c
 * @author  Dominik Widhalm
 * @version 1.3.0
 * @date    2022/01/21
 */


/***** INCLUDES *******************************************************/
#include "xbee.h"


/***** GLOBAL VARIABLES ***********************************************/
/*! Xbee sleep request pin GPIO structure */
hw_io_t xbee_sleep_req;
/*! Xbee sleep indicator pin GPIO structure */
hw_io_t xbee_sleep_ind;
/*! Frame ID (start with 1; 0 is reserved) */
uint8_t fid_cur = 1;
/*! Function callback for writing a byte */
void (*_write)(uint8_t byte) = NULL;
/*! Function callback for reading a byte */
int8_t (*_read)(uint8_t* byte) = NULL;
/*! Function callback for rx data available check */
uint8_t (*_available)(void) = NULL;


/***** LOCAL FUNCTION PROTOTYPES **************************************/
/* Local AT command */
static XBEE_RET_t _at_local_write(char* command, uint64_t value, uint8_t fid);
static XBEE_RET_t _at_local_query(char* command, uint8_t fid);
static XBEE_RET_t _at_local_response(uint64_t* value, uint8_t* fid);
/* Remote AT command */
static XBEE_RET_t _at_remote_write(uint64_t mac, uint16_t addr, char* command, uint64_t value, uint8_t fid);
static XBEE_RET_t _at_remote_query(uint64_t mac, uint16_t addr, char* command, uint8_t fid);
static XBEE_RET_t _at_remote_response(uint64_t* mac, uint16_t* addr, uint64_t* value, uint8_t* fid);


/***** FUNCTIONS ******************************************************/
/*!
 * Initialize the UART interface for Xbee communication.
 *
 * @param[in]   write       Function callback for writing data
 * @param[in]   read        Function callback for reading data
 * @param[in]   available   Function callback for data available check
 */
void xbee_init(void (*write)(uint8_t byte), int8_t (*read)(uint8_t* byte), uint8_t (*available)(void)) {
    /* Set the function callbacks */
    _write = write;
    _read = read;
    _available = available;
    /* (Re)set the Frame ID */
    fid_reset();
    /* Fill the sleep signal GPIO structures (default) */
    hw_get_io(&xbee_sleep_req, &XBEE_SLEEP_REQ_DDR, &XBEE_SLEEP_REQ_PORT, &XBEE_SLEEP_REQ_PIN, XBEE_SLEEP_REQ_GPIO);
    hw_get_io(&xbee_sleep_ind, &XBEE_SLEEP_IND_DDR, &XBEE_SLEEP_IND_PORT, &XBEE_SLEEP_IND_PIN, XBEE_SLEEP_IND_GPIO);
    /* Set GPIO input/output and level (initially enabled) */
    hw_set_output_low(&xbee_sleep_req);
    hw_set_output(&xbee_sleep_req);
    hw_set_input(&xbee_sleep_ind);
}


/*!
 * Set the sleep request gpio (SLEEP_RQ) in case not the default shall be used.
 *
 * @param[in]   ddr     Pointer to the GPIO's DDRx register
 * @param[in]   port    Pointer to the GPIO's PORTx register
 * @param[in]   pin     Pointer to the GPIO's PINx register
 * @param[in]   portpin Index of the GPIO pin
 */
void xbee_set_sleep_request_gpio(volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin) {
    /* Call the respective HW function */
    hw_get_io(&xbee_sleep_req, ddr, port, pin, portpin);
}


/*!
 * Set the sleep indicator gpio (SLEEP) in case not the default shall be used.
 *
 * @param[in]   ddr     Pointer to the GPIO's DDRx register
 * @param[in]   port    Pointer to the GPIO's PORTx register
 * @param[in]   pin     Pointer to the GPIO's PINx register
 * @param[in]   portpin Index of the GPIO pin
 */
void xbee_set_sleep_indicator_gpio(volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin) {
    /* Call the respective HW function */
    hw_get_io(&xbee_sleep_ind, ddr, port, pin, portpin);
}


/*!
 * Request the xbee to enter sleep mode
 *
 * @return      OK in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_sleep_enable(void) {
    /* Request xbee sleep */
    hw_set_output_high(&xbee_sleep_req);
    /* Check xbee's response */
    uint16_t timeout = XBEE_WAKE_TIMEOUT * (1000UL / XBEE_WAKE_TIMEOUT_DELAY);
    while(timeout--) {
        /* Check sleep indicator pin state */
        if(hw_read_input(&xbee_sleep_ind) == HW_STATE_LOW) {
            /* Sleep request successful */
            return XBEE_RET_OK;
        }
        /* Wait for some time */
        _delay_ms(XBEE_WAKE_TIMEOUT_DELAY);
    }
    /* Sleep request failed */
    return XBEE_RET_TIMEOUT;
}


/*!
 * Request the xbee to wake-up from sleep mode
 *
 * @return      OK in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_sleep_disable(void) {
    /* Request xbee wake-up */
    hw_set_output_low(&xbee_sleep_req);
    /* Check xbee's response */
    uint16_t timeout = XBEE_WAKE_TIMEOUT * (1000UL / XBEE_WAKE_TIMEOUT_DELAY);
    while(timeout--) {
        /* Check sleep indicator pin state */
        if(hw_read_input(&xbee_sleep_ind) == HW_STATE_HIGH) {
            /* Wake-up successful */
            return XBEE_RET_OK;
        }
        /* Wait for some time */
        _delay_ms(XBEE_WAKE_TIMEOUT_DELAY);
    }
    /* Wake-up failed */
    return XBEE_RET_TIMEOUT;
}


/*!
 * Reset the frame ID counter.
 */
void fid_reset(void) {
    /* Reset the frame ID counter to 1 (0 is reserved) */
    fid_cur = 1;
}


/*!
 * Reset the frame ID counter.
 */
uint8_t fid_get_next(void) {
    uint8_t tmp = fid_cur;
    /* Increment the frame ID counter */
    fid_cur++;
    /* Check if frame ID counter is 0 */
    if(fid_cur==0) {
        /* Set to 1 (0 is reserved) */
        fid_cur = 1;
    }
    /* Return the former frame ID */
    return tmp;
}


/*!
 * Write a local AT command.
 *
 * @param[in]   command     AT command (two character)
 * @param[in]   value       Value to be assigned
 * @param[in]   fid         Frame ID.
 * @return      OK in case of success; ERROR otherwise
 */
static XBEE_RET_t _at_local_write(char* command, uint64_t value, uint8_t fid) {
    /* Temporary variables for frame length (depends on value) */
    uint16_t len;
    /* Data array for the frame (max. length: 16 bytes) */
    uint8_t data[16] = {0};

    /* Calculate length (#bytes between length and checksum) */
    if(value > 0x00FFFFFFFF) {
        /* 64-bit: Data length will be 12 byte */
        len = 12;
    } else if(value > 0x00FFFF) {
        /* 32-bit: Data length will be 8 byte */
        len = 8;
    } else if(value > 0x00FF) {
        /* 16-bit: Data length will be 6 byte */
        len = 6;
    } else {
        /* 8-bit: Data length will be 5 byte */
        len = 5;
    }
    
    /* 0. byte is the Start Delimiter */
    data[0] = XBEE_START_DELIMITER;
    /* 1. and 2. byte is the length */
    data[1] = (len >> 8) & 0xFF;
    data[2] = len & 0xFF;
    /* 3. byte is the frame type */
    data[3] = XBEE_FRAME_ATCMD_LOCAL;
    /* 4. byte is the frame ID */
    data[4] = fid;
    /* 5. and 6. byte is the AT command */
    data[5] = command[0];
    data[6] = command[1];
    /* 7. to n byte is the value */
    switch(len) {
        case 12:
            /* 64-bit value */
            data[7]  = (value >> 56) & 0xFF;
            data[8]  = (value >> 48) & 0xFF;
            data[9]  = (value >> 40) & 0xFF;
            data[10] = (value >> 32) & 0xFF;
            data[11] = (value >> 24) & 0xFF;
            data[12] = (value >> 16) & 0xFF;
            data[13] = (value >> 8)  & 0xFF;
            data[14] = value & 0xFF;
            break;
        case 8:
            /* 32-bit value */
            data[7]  = (value >> 24) & 0xFF;
            data[8]  = (value >> 16) & 0xFF;
            data[9]  = (value >> 8)  & 0xFF;
            data[10] = value & 0xFF;
            break;
        case 6:
            /* 16-bit value */
            data[7]  = (value >> 8)  & 0xFF;
            data[8]  = value & 0xFF;
            break;
        case 5:
            /* 8-bit value */
            data[7]  = value & 0xFF;
            break;
        default:
            /* Should never happen */
            return XBEE_RET_ERROR;
    }
    /* Last byte is the CRC (ignore first 3 bytes) */
    data[len+3] = xbee_get_crc(&data[3], len);
    /* Write the data byte by byte */
    for(uint8_t i=0; i<(len+4); i++) {
        /* Write the byte via the callback function */
        _write(data[i]);
    }
    /* Done */
    return XBEE_RET_OK;
}


/*!
 * Query a local AT command.
 *
 * @param[in]   command     AT command (two character)
 * @param[in]   fid         Frame ID.
 * @return      OK in case of success; ERROR otherwise
 */
static XBEE_RET_t _at_local_query(char* command, uint8_t fid) {
    /* Data array for the frame (length: 8 bytes) */
    uint8_t data[8] = {0};
    /* 0. byte is the Start Delimiter */
    data[0] = XBEE_START_DELIMITER;
    /* 1. and 2. byte is the length (fixed 4 byte) */
    data[1] = (4 >> 8) & 0xFF;
    data[2] = 4 & 0xFF;
    /* 3. byte is the frame type */
    data[3] = XBEE_FRAME_ATCMD_LOCAL;
    /* 4. byte is the frame ID */
    data[4] = fid;
    /* 5. and 6. byte is the AT command */
    data[5] = command[0];
    data[6] = command[1];
    /* Last byte is the CRC (ignore first 3 bytes) */
    data[7] = xbee_get_crc(&data[3], 4);
    /* Write the data byte by byte */
    for(uint8_t i=0; i<8; i++) {
        /* Write the byte via the callback function */
        _write(data[i]);
    }
    /* Done */
    return XBEE_RET_OK;
}


/*!
 * Get the response to a local AT command.
 *
 * @param[out]  value       Value returned by the command
 * @param[out]  fid         Frame ID returned.
 * @return      Size of the command value in case of success; ERROR otherwise
 */
static XBEE_RET_t _at_local_response(uint64_t* value, uint8_t* fid) {
    /* Data array for the frame (max. length: 17 bytes) */
    uint8_t data[17] = {0};
    /* Number of bytes of the response value received */
    uint8_t resp_cnt = 0;
    /* Length of the frame to be received */
    uint16_t len = 0;
    /* Reading position (index) */
    uint8_t pos = 0;
    /* Flag for completed response */
    uint8_t complete = 0;
    
    /*** Read data ***/
    uint16_t timeout = XBEE_RESPONSE_TIMEOUT * (1000UL / XBEE_RESPONSE_TIMEOUT_DELAY);
    do {
        /* Check if data is available to be received */
        if(_available()) {
            /* Read next byte */
            if(_read(&data[pos]) != 0) {
                /* Read failed */
                continue;
            }
            /* Store/process data based on frame position (index) */
            switch(pos) {
                /* First byte should be the start delimiter */
                case 0:
                    /* Check if current byte is the start delimiter */
                    if(data[pos] == XBEE_START_DELIMITER) {
                        /* Increment position */
                        pos++;
                    }
                    /* Store value */
                    break;
                /* Second byte should be the length MSB */
                case 1:
                    /* Add MSB value to len variable */
                    len |= (uint16_t)data[pos] << 8;
                    /* Increment position */
                    pos++;
                    
                    break;
                /* Third byte should be the length LSB */
                case 2:
                    /* Add LSB value to len variable */
                    len |= (uint16_t)data[pos];
                    /* Increment position */
                    pos++;
                    break;
                /* All other bytes ... */
                default:
                    /* Check if maximum frame data size has been exceeded */
                    if(pos > (17-1)) {
                        /* Size limit exceed */
                        return XBEE_RET_PAYLOAD_SIZE_EXCEEDED;
                    }
                    /* Check if the end of the frame has been reached */
                    /* Packet length does not include start, length, or checksum bytes -> add 3 */
                    if(pos == (len+3)) {
                        /* Reception done ... continue with data processing */
                        complete = 1;
                        break;
                    } else {
                        /* Increment position */
                        pos++;
                    }
                    
                    break;
            }
        }
        /* Wait for some time */
        _delay_ms(XBEE_RESPONSE_TIMEOUT_DELAY);
    } while((--timeout) && (complete==0));
    /* Check if timeout has triggered */
    if(timeout == 0) {
        /* Response timed out */
        return XBEE_RET_TIMEOUT;
    }
    
    /*** Process data ***/
    /* Check the frame type */
    if(data[3] != XBEE_FRAME_ATCMD_LOCAL_RESP) {
        /* Frame type does not match */
        return XBEE_RET_INVALID_FRAME;
    }
    /* Check the response status */
    switch(data[7]) {
        case 0x00:
            /* Everything OK */
            break;
        case 0x02:
            /* Invalid command */
            return XBEE_RET_INVALID_CMD;
        case 0x03:
            /* Invalid parameter */
            return XBEE_RET_INVALID_PARAM;
        default:
            /* Some ERROR */
            return XBEE_RET_ERROR;
    }
    /* Remained depends on command data size */
    switch(len-5) {
        case 0:         // No command data
            /* No command data -> check CRC */
            if(xbee_check_crc(&data[3], 5, data[8]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 0 bytes value */
            resp_cnt = 0;
            break;
        case 1:         // 8-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 6, data[9]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 1 command data byte */
            *value = data[8];
            resp_cnt = 1;
            break;
        case 2:         // 16-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 7, data[10]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 2 command data byte */
            *value = ((uint16_t)data[8]<<8);
            *value |= (uint16_t)data[9];
            resp_cnt = 2;
            break;
        case 3:         // 24-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 8, data[11]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 3 command data byte */
            *value = ((uint32_t)data[8]<<16);
            *value |= ((uint32_t)data[9]<<8);
            *value |= (uint32_t)data[10];
            resp_cnt = 3;
            break;
        case 4:         // 32-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 9, data[12]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 4 command data byte */
            *value = ((uint32_t)data[8]<<24);
            *value |= ((uint32_t)data[9]<<16);
            *value |= ((uint32_t)data[10]<<8);
            *value |= (uint32_t)data[11];
            resp_cnt = 4;
            break;
        case 5:         // 40-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 10, data[13]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 5 command data byte */
            *value = ((uint64_t)data[8]<<32);
            *value |= ((uint64_t)data[9]<<24);
            *value |= ((uint64_t)data[10]<<16);
            *value |= ((uint64_t)data[11]<<8);
            *value |= (uint64_t)data[12];
            resp_cnt = 5;
            break;
        case 6:         // 48-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 11, data[14]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 6 command data byte */
            *value = ((uint64_t)data[8]<<40);
            *value |= ((uint64_t)data[9]<<32);
            *value |= ((uint64_t)data[10]<<24);
            *value |= ((uint64_t)data[11]<<16);
            *value |= ((uint64_t)data[12]<<8);
            *value |= (uint64_t)data[13];
            resp_cnt = 6;
            break;
        case 7:         // 56-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 12, data[15]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 7 command data byte */
            *value = ((uint64_t)data[8]<<48);
            *value |= ((uint64_t)data[9]<<40);
            *value |= ((uint64_t)data[10]<<32);
            *value |= ((uint64_t)data[11]<<24);
            *value |= ((uint64_t)data[12]<<16);
            *value |= ((uint64_t)data[13]<<8);
            *value |= (uint64_t)data[14];
            resp_cnt = 7;
            break;
        case 8:         // 64-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 13, data[16]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 8 command data byte */
            *value = ((uint64_t)data[8]<<56);
            *value |= ((uint64_t)data[9]<<48);
            *value |= ((uint64_t)data[10]<<40);
            *value |= ((uint64_t)data[11]<<32);
            *value |= ((uint64_t)data[12]<<24);
            *value |= ((uint64_t)data[13]<<16);
            *value |= ((uint64_t)data[14]<<8);
            *value |= (uint64_t)data[15];
            resp_cnt = 8;
            break;
        default:
            /* That should not happen */
            return XBEE_RET_ERROR;
    }
    /* Get the frame id */
    *fid = data[4];
    /* Done - Return the size of the command value */
    return resp_cnt;
}


/*!
 * Write a local AT command and check the response.
 *
 * @param[in]   command     AT command (two character)
 * @param[in]   value       Value for the command
 * @return      Size of the command value in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_at_local_cmd_write(char* command, uint64_t value){
    int8_t ret = XBEE_RET_OK;
    uint8_t fid_ret = 0;
    /* Get the ID for the current frame */
    uint8_t fid = fid_get_next();
    /* There should be no response value, still ... */
    uint64_t resp;
    
    /* Send the local AT command with the new value */
    ret = _at_local_write(command, value, fid);
    if(ret < XBEE_RET_OK) {
        /* Sending failed */
        return ret;
    }
    
    /* Check the response */
    ret = _at_local_response(&resp, &fid_ret);
    if(ret < XBEE_RET_OK) {
        /* Response error */
        return ret;
    }
    
    /* Check if the FIDs match */
    if(fid != fid_ret) {
        /* FIDs do not match */
        return XBEE_RET_FID_NOT_MATCH;
    }
    
    /* Done - Return the size of the command value */
    return ret;
}


/*!
 * Read the response to a local AT command.
 *
 * @param[in]   command     AT command (two character)
 * @param[out]  value       Value returned by the command
 * @return      Size of the command value in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_at_local_cmd_read(char* command, uint64_t* value) {
    int8_t ret = XBEE_RET_OK;
    uint8_t fid_ret = 0;
    /* Get the ID for the current frame */
    uint8_t fid = fid_get_next();
    
    /* Send the local AT command */
    ret = _at_local_query(command, fid);
    if(ret != XBEE_RET_OK) {
        /* Sending failed */
        return ret;
    }
    
    /* Check the response */
    ret = _at_local_response(value, &fid_ret);
    if(ret < XBEE_RET_OK) {
        /* Response error */
        return ret;
    }
    
    /* Check if the FIDs match */
    if(fid != fid_ret) {
        /* FIDs do not match */
        return XBEE_RET_FID_NOT_MATCH;
    }
    
    /* Done - Return the size of the command value */
    return ret;
}


/*!
 * Write a remote AT command.
 * Remote target specified by MAC or 16-bit address.
 *
 * @param[in]   mac         MAC address
 * @param[in]   addr        16-bit address
 * @param[in]   command     AT command (two character)
 * @param[in]   value       Value for the command
 * @param[in]   fid         Frame ID.
 * @return      OK in case of success; ERROR otherwise
 */
static XBEE_RET_t _at_remote_write(uint64_t mac, uint16_t addr, char* command, uint64_t value, uint8_t fid) {
    uint8_t i;
    /* Temporary variables for frame length (depends on value) */
    uint16_t len=0;
    /* Data array for the frame (max. length: 27 bytes) */
    uint8_t data[27] = {0};

    /* Calculate length (#bytes between length and checksum) */
    if(value > 0x00FFFFFFFF) {
        /* 64-bit: Data length will be 23 byte */
        len = 23;
    } else if(value > 0x00FFFF) {
        /* 32-bit: Data length will be 19 byte */
        len = 19;
    } else if(value > 0x00FF) {
        /* 16-bit: Data length will be 17 byte */
        len = 17;
    } else {
        /* 8-bit: Data length will be 16 byte */
        len = 16;
    }
    
    /* 0. byte is the Start Delimiter */
    data[0] = XBEE_START_DELIMITER;
    /* 1. and 2. byte is the length */
    data[1] = (len >> 8) & 0xFF;
    data[2] = len & 0xFF;
    /* 3. byte is the frame type */
    data[3] = XBEE_FRAME_ATCMD_REMOTE;
    /* 4. byte is the frame ID */
    data[4] = fid;
    /* 5. to 12. byte is the MAC address */
    for(i=0; i<8; i++) {
        data[5+i] = (mac >> ((7-i)*8)) & 0xFF;
    }
    /* 13. and 14. byte is the 16-bit address */
    data[13] = (addr >> 8) & 0xFF;
    data[14] = addr & 0xFF;
    /* 15. byte are the remote command options */
    data[15] = XBEE_ATCMD_REMOTE_OPT;
    /* 16. and 17. byte is the AT command */
    data[16] = command[0];
    data[17] = command[1];
    /* 18. to n byte is the value */
    switch(len) {
        case 23:
            /* 64-bit value */
            data[18] = (value >> 56) & 0xFF;
            data[19] = (value >> 48) & 0xFF;
            data[20] = (value >> 40) & 0xFF;
            data[21] = (value >> 32) & 0xFF;
            data[22] = (value >> 24) & 0xFF;
            data[23] = (value >> 16) & 0xFF;
            data[24] = (value >> 8) & 0xFF;
            data[25] = value & 0xFF;
            break;
        case 19:
            /* 32-bit value */
            data[18] = (value >> 24) & 0xFF;
            data[19] = (value >> 16) & 0xFF;
            data[20] = (value >> 8) & 0xFF;
            data[21] = value & 0xFF;
            break;
        case 18:
            /* 24-bit value */
            data[18] = (value >> 16) & 0xFF;
            data[19] = (value >> 8) & 0xFF;
            data[20] = value & 0xFF;
            break;
        case 17:
            /* 16-bit value */
            data[18] = (value >> 8) & 0xFF;
            data[19] = value & 0xFF;
            break;
        case 16:
            /* 8-bit value */
            data[18] = value & 0xFF;
            break;
        default:
            /* Should never happen */
            return XBEE_RET_ERROR;
    }
    /* Last byte is the CRC (ignore first 3 bytes) */
    data[len+3] = xbee_get_crc(&data[3], len);
    /* Write the data byte by byte */
    for(i=0; i<(len+4); i++) {
        /* Write the byte via the callback function */
        _write(data[i]);
    }
    /* Done */
    return XBEE_RET_OK;
}


/*!
 * Query a remote AT command.
 * Remote target specified by MAC or 16-bit address.
 *
 * @param[in]   mac         MAC address
 * @param[in]   addr        16-bit address
 * @param[in]   command     AT command (two character)
 * @param[in]   fid         Frame ID.
 * @return      OK in case of success; ERROR otherwise
 */
static XBEE_RET_t _at_remote_query(uint64_t mac, uint16_t addr, char* command, uint8_t fid) {
    uint8_t i=0;
    /* Data array for the frame (max. length: 23 bytes) */
    uint8_t data[19] = {0};
    
    /* 0. byte is the Start Delimiter */
    data[0] = XBEE_START_DELIMITER;
    /* 1. and 2. byte is the length */
    data[1] = (15 >> 8) & 0xFF;
    data[2] = 15 & 0xFF;
    /* 3. byte is the frame type */
    data[3] = XBEE_FRAME_ATCMD_REMOTE;
    /* 4. byte is the frame ID */
    data[4] = fid;
    /* 5. to 12. byte is the MAC address */
    for(i=0; i<8; i++) {
        data[5+i] = (mac >> ((7-i)*8)) & 0xFF;
    }
    /* 13. and 14. byte is the 16-bit address */
    data[13] = (addr >> 8) & 0xFF;
    data[14] = addr & 0xFF;
    /* 15. byte are the remote command options */
    data[15] = XBEE_ATCMD_REMOTE_OPT;
    /* 16. and 17. byte is the AT command */
    data[16] = command[0];
    data[17] = command[1];
    /* Last byte is the CRC (ignore first 3 bytes) */
    data[18] = xbee_get_crc(&data[3], 15);
    /* Write the data byte by byte */
    for(i=0; i<19; i++) {
        /* Write the byte via the callback function */
        _write(data[i]);
    }
    /* Done */
    return XBEE_RET_OK;
}


/*!
 * Get the response to a remote AT command.
 * Remote target specified by MAC or 16-bit address.
 *
 * @param[in]   mac         MAC address
 * @param[in]   addr        16-bit address
 * @param[out]  value       Value for the command
 * @param[out]  fid         Frame ID returned.
 * @return      Size of the command value in case of success; ERROR otherwise
 */
static XBEE_RET_t _at_remote_response(uint64_t* mac, uint16_t* addr, uint64_t* value, uint8_t* fid) {
    uint8_t i;
    /* Data array for the frame (max. length: 27 bytes) */
    uint8_t data[27] = {0};
    /* Number of bytes of the response value received */
    uint8_t resp_cnt = 0;
    /* Temporary variables for frame length (depends on value) and timeout */
    uint16_t len;
    /* Reading position (index) */
    uint8_t pos = 0;
    /* Flag for completed response */
    uint8_t complete = 0;
    
    /*** Read data ***/
    uint16_t timeout = XBEE_RESPONSE_TIMEOUT * (1000UL / XBEE_RESPONSE_TIMEOUT_DELAY);
    do {
        /* Check if data is available to be received */
        if(_available()) {
            /* Read next byte */
            if(_read(&data[pos]) != 0) {
                /* Read failed */
                continue;
            }
            /* Store/process data based on frame position (index) */
            switch(pos) {
                /* First byte should be the start delimiter */
                case 0:
                    /* Check if current byte is the start delimiter */
                    if(data[pos] == XBEE_START_DELIMITER) {
                        /* Increment position */
                        pos++;
                    }
                    /* Store value */
                    break;
                /* Second byte should be the length MSB */
                case 1:
                    /* Add MSB value to len variable */
                    len |= (uint16_t)data[pos] << 8;
                    /* Increment position */
                    pos++;
                    
                    break;
                /* Third byte should be the length LSB */
                case 2:
                    /* Add LSB value to len variable */
                    len |= (uint16_t)data[pos];
                    /* Increment position */
                    pos++;
                    
                    break;
                /* All other bytes ... */
                default:
                    /* Check if maximum frame data size has been exceeded */
                    if(pos > (27-1)) {
                        /* Size limit exceed */
                        return XBEE_RET_PAYLOAD_SIZE_EXCEEDED;
                    }
                    /* Check if the end of the frame has been reached */
                    /* Packet length does not include start, length, or checksum bytes -> add 3 */
                    if(pos == (len+3)) {
                        /* Reception done ... continue with data processing */
                        complete = 1;
                        break;
                    } else {
                        /* Increment position */
                        pos++;
                    }
                    
                    break;
            }
        }
        /* Wait for some time */
        _delay_ms(XBEE_RESPONSE_TIMEOUT_DELAY);
    } while((--timeout) && (complete==0));
    /* Check if timeout has triggered */
    if(timeout == 0) {
        /* Response timed out */
        return XBEE_RET_TIMEOUT;
    }
    
    /*** Process data ***/
    /* Check the frame type */
    if(data[3] != XBEE_FRAME_ATCMD_REMOTE_RESP) {
        /* Frame type does not match */
        return XBEE_RET_INVALID_FRAME;
    }
    /* Check the response status */
    switch(data[17]) {
        case 0x00:
            /* Everything OK */
            break;
        case 0x02:
            /* Invalid command */
            return XBEE_RET_INVALID_CMD;
        case 0x03:
            /* Invalid parameter */
            return XBEE_RET_INVALID_PARAM;
        case 0x04:
            /* Transmission failure */
            return XBEE_RET_FAILED_TRANSMISSION;
        case 0x0B:
            /* No Secure Session */
            return XBEE_RET_NOT_SECURE;
        case 0x0C:
            /* Encryption error */
            return XBEE_RET_ENCRYPTION_ERROR;
        case 0x0D:
            /* Command was sent insecurely */
            return XBEE_RET_CMD_NOT_SECURE;
        default:
            /* Some ERROR */
            return XBEE_RET_ERROR;
    }
    /* Remained depends on command data size */
    switch(len-15) {
        case 0:         // No command data
            /* No command data -> check CRC */
            if(xbee_check_crc(&data[3], 15, data[18]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 0 command data bytes */
            resp_cnt = 0;
            break;
        case 1:         // 8-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 16, data[19]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 1 command data byte */
            *value = data[18];
            resp_cnt = 1;
            break;
        case 2:         // 16-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 17, data[20]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 2 command data byte */
            *value = ((uint16_t)data[18]<<8);
            *value |= (uint16_t)data[19];
            resp_cnt = 2;
            break;
        case 3:         // 24-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 18, data[21]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 3 command data byte */
            *value |= ((uint32_t)data[18]<<16);
            *value |= ((uint32_t)data[19]<<8);
            *value |= (uint32_t)data[20];
            resp_cnt = 3;
            break;
        case 4:         // 32-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 19, data[22]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 4 command data byte */
            *value = ((uint32_t)data[18]<<24);
            *value |= ((uint32_t)data[19]<<16);
            *value |= ((uint32_t)data[20]<<8);
            *value |= (uint32_t)data[21];
            resp_cnt = 4;
            break;
        case 5:         // 40-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 20, data[23]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 5 command data byte */
            *value = ((uint64_t)data[18]<<32);
            *value |= ((uint64_t)data[19]<<24);
            *value |= ((uint64_t)data[20]<<16);
            *value |= ((uint64_t)data[21]<<8);
            *value |= (uint64_t)data[22];
            resp_cnt = 5;
            break;
        case 6:         // 48-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 21, data[24]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 6 command data byte */
            *value = ((uint64_t)data[18]<<40);
            *value |= ((uint64_t)data[19]<<32);
            *value |= ((uint64_t)data[20]<<24);
            *value |= ((uint64_t)data[21]<<16);
            *value |= ((uint64_t)data[22]<<8);
            *value |= (uint64_t)data[23];
            resp_cnt = 6;
            break;
        case 7:         // 64-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 22, data[25]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 7 command data byte */
            *value = ((uint64_t)data[18]<<48);
            *value |= ((uint64_t)data[19]<<40);
            *value |= ((uint64_t)data[20]<<32);
            *value |= ((uint64_t)data[21]<<24);
            *value |= ((uint64_t)data[22]<<16);
            *value |= ((uint64_t)data[23]<<8);
            *value |= (uint64_t)data[24];
            resp_cnt = 7;
            break;
        case 8:         // 64-bit command data
            /* Check the CRC */
            if(xbee_check_crc(&data[3], 23, data[26]) != XBEE_RET_OK) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 8 command data byte */
            *value = ((uint64_t)data[18]<<56);
            *value |= ((uint64_t)data[19]<<48);
            *value |= ((uint64_t)data[20]<<40);
            *value |= ((uint64_t)data[21]<<32);
            *value |= ((uint64_t)data[22]<<24);
            *value |= ((uint64_t)data[23]<<16);
            *value |= ((uint64_t)data[24]<<8);
            *value |= (uint64_t)data[25];
            resp_cnt = 8;
            break;
        default:
            /* That should not happen */
            return XBEE_RET_ERROR;
    }
    /* Get the frame id */
    *fid = data[4];
    /* Get the sender's MAC */
    *mac = 0;
    for(i=0; i<8; i++) {
        *mac |= ((uint64_t)data[5+i]<<((7-i)*8));
    }
    /* Get the sender's 16-bit address */
    *addr = ((uint16_t)data[13] << 8) | (uint16_t)data[14];
    
    /* Done - Return the size of the command value */
    return resp_cnt;
}


/*!
 * Write a remote AT command and check the response.
 *
 * @param[in]   mac         MAC address
 * @param[in]   addr        16-bit address
 * @param[in]   command     AT command (two character)
 * @param[in]   value       Value for the command
 * @return      Size of the command value in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_at_remote_cmd_write(uint64_t mac, uint16_t addr, char* command, uint64_t value) {
    int8_t ret = XBEE_RET_OK;
    uint8_t fid_ret = 0;
    uint16_t addr_ret = 0;
    uint64_t mac_ret = 0;
    /* Get the ID for the current frame */
    uint8_t fid = fid_get_next();
    /* There should be no response value, still ... */
    uint64_t resp = 0;
    
    /* Send the remote AT command */
    ret = _at_remote_write(mac, addr, command, value, fid);
    if(ret < XBEE_RET_OK) {
        /* Sending failed */
        return ret;
    }
    
    /* Check the response */
    ret = _at_remote_response(&mac_ret, &addr_ret, &resp, &fid_ret);
    if(ret < XBEE_RET_OK) {
        /* Response error */
        return ret;
    }
    
    /* Check if the FIDs match */
    if(fid != fid_ret) {
        /* FIDs do not match */
        return XBEE_RET_FID_NOT_MATCH;
    }
    
    /* Check if the MACs match */
    if(mac != XBEE_ADDR64_USE16) {
        if(mac != mac_ret) {
            /* MACs do not match */
            return XBEE_RET_MAC_NOT_MATCH;
        }
    }
    
    /* Check if the 16-bit addresses match */
    if(addr != XBEE_ADDR16_USE64) {
        if(addr != addr_ret) {
            /* ADDRs do not match */
            return XBEE_RET_ADDR_NOT_MATCH;
        }
    }
    
    /* Done - Return the size of the command value */
    return ret;
}


/*!
 * Read the response to a remote AT command.
 *
 * @param[in]   mac         MAC address
 * @param[in]   addr        16-bit address
 * @param[in]   command     AT command (two character)
 * @param[out]  value       Value returned by the command
 * @return      Size of the command value in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_at_remote_cmd_read(uint64_t mac, uint16_t addr, char* command, uint64_t* value) {
    int8_t ret = XBEE_RET_OK;
    uint8_t fid_ret = 0;
    uint16_t addr_ret = 0;
    uint64_t mac_ret = 0;
    /* Get the ID for the current frame */
    uint8_t fid = fid_get_next();
        
    /* Send the remote AT command */
    ret = _at_remote_query(mac, addr, command, fid);
    if(ret < XBEE_RET_OK) {
        /* Sending failed */
        return ret;
    }
    
    /* Check the response */
    ret = _at_remote_response(&mac_ret, &addr_ret, value, &fid_ret);
    if(ret < XBEE_RET_OK) {
        /* Response error */
        return ret;
    }
    
    /* Check if the FIDs match */
    if(fid != fid_ret) {
        /* FIDs do not match */
        return XBEE_RET_FID_NOT_MATCH;
    }
    
    /* Check if the MACs match */
    if(mac != XBEE_ADDR64_USE16) {
        if(mac != mac_ret) {
            /* MACs do not match */
            return XBEE_RET_MAC_NOT_MATCH;
        }
    }
    
    /* Check if the 16-bit addresses match */
    if(addr != XBEE_ADDR16_USE64) {
        if(addr != addr_ret) {
            /* ADDRs do not match */
            return XBEE_RET_ADDR_NOT_MATCH;
        }
    }
    
    /* Done - Return the size of the command value */
    return ret;
}


/*!
 * Transmit a specified number of bytes to the destination.
 * Destination target specified by MAC or 16-bit address.
 *
 * @param[in]   mac         MAC address
 * @param[in]   addr        16-bit address
 * @param[in]   payload     Payload data to be transmitted
 * @param[in]   cnt         Number of payload bytes
 * @param[in]   fid         Frame ID.
 * @return      OK in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_transmit(uint64_t mac, uint16_t addr, uint8_t* payload, uint16_t cnt, uint8_t fid) {
    uint8_t i=0;
    /* Temporary variables for frame length (depends on cnt) */
    uint16_t len = cnt + 14;
    /* Data array for the frame (payload + overhead) */
    uint8_t data[XBEE_CONF_PAYLOAD_MAX+14] = {0};
    
    /* Check if payload size does not exceed limits */
    if(cnt > XBEE_CONF_PAYLOAD_MAX) {
        /* Payload too big */
        return XBEE_RET_PAYLOAD_SIZE_EXCEEDED;
    }
    
    /* 0. byte is the Start Delimiter */
    data[0] = XBEE_START_DELIMITER;
    /* 1. and 2. byte is the length */
    data[1] = (len >> 8) & 0xFF;
    data[2] = len & 0xFF;
    /* 3. byte is the frame type */
    data[3] = XBEE_FRAME_TRANSMIT;
    /* 4. byte is the frame ID */
    data[4] = fid;
    /* 5. to 12. byte is the MAC address */
    for(i=0; i<8; i++) {
        data[5+i] = (mac >> ((7-i)*8)) & 0xFF;
    }
    /* 13. and 14. byte is the 16-bit address */
    data[13] = (addr >> 8) & 0xFF;
    data[14] = addr & 0xFF;
    /* 15. byte is the broadcast radius */
    data[15] = XBEE_TRANSMIT_BROADCAST_RADIUS;
    /* 16. byte are the transmit command options */
    data[16] = XBEE_TRANSMIT_OPT;
    /* 17. to n byte is tha payload */
    for(i=0; i<cnt; i++) {
        data[17+i] = payload[i];
    }
    /* Last byte is the CRC (ignore first 3 bytes) */
    data[len+3] = xbee_get_crc(&data[3], len);
    /* Write the data byte by byte */
    for(i=0; i<(len+4); i++) {
        /* Write the byte via the callback function */
        _write(data[i]);
    }
    /* Done */
    return XBEE_RET_OK;
}


/*!
 * Get the transmit response status.
 *
 * @param[out]  delivery    Delivery response status
 * @return      OK in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_transmit_status(uint8_t* delivery) {
    /* Data array for the frame (max. length: 23 bytes) */
    uint8_t data[11] = {0};
    /* Length of the frame to be received */
    uint16_t len = 0;
    /* Reading position (index) */
    uint8_t pos = 0;
    /* Flag for completed response */
    uint8_t complete = 0;
    
    /*** Read data ***/
    uint16_t timeout = XBEE_RESPONSE_TIMEOUT * (1000UL / XBEE_RESPONSE_TIMEOUT_DELAY);
    do {
        /* Check if data is available to be received */
        if(_available()) {
            /* Read next byte */
            if(_read(&data[pos]) != 0) {
                /* Read failed */
                continue;
            }
            /* Store/process data based on frame position (index) */
            switch(pos) {
                /* First byte should be the start delimiter */
                case 0:
                    /* Check if current byte is the start delimiter */
                    if(data[pos] == XBEE_START_DELIMITER) {
                        /* Increment position */
                        pos++;
                    }
                    /* Store value */
                    break;
                /* Second byte should be the length MSB */
                case 1:
                    /* Add MSB value to len variable */
                    len |= (uint16_t)data[pos] << 8;
                    /* Increment position */
                    pos++;
                    
                    break;
                /* Third byte should be the length LSB */
                case 2:
                    /* Add LSB value to len variable */
                    len |= (uint16_t)data[pos];
                    /* Increment position */
                    pos++;
                    
                    break;
                /* All other bytes ... */
                default:
                    /* Check if maximum frame data size has been exceeded */
                    if(pos > (11-1)) {
                        /* Size limit exceed */
                        return XBEE_RET_PAYLOAD_SIZE_EXCEEDED;
                    }
                    /* Check if the end of the frame has been reached */
                    /* Packet length does not include start, length, or checksum bytes -> add 3 */
                    if(pos == (len+3)) {
                        /* Reception done ... continue with data processing */
                        complete = 1;
                        break;
                    } else {
                        /* Increment position */
                        pos++;
                    }
                    
                    break;
            }
        }
        /* Wait for some time */
        _delay_ms(XBEE_RESPONSE_TIMEOUT_DELAY);
    } while((--timeout) && (complete==0));
    /* Check if timeout has triggered */
    if(timeout == 0) {
        /* Response timed out */
        return XBEE_RET_TIMEOUT;
    }
    
    /*** Process data ***/
    /* Check the frame type */
    if(data[3] != XBEE_FRAME_TRANSMIT_STATUS) {
        /* Frame type does not match */
        return XBEE_RET_INVALID_FRAME;
    }
    /* Check the CRC */
    if(xbee_check_crc(&data[3], 7, data[10]) != XBEE_RET_OK) {
        return XBEE_RET_INVALID_CRC;
    }
    /* Get the Delivery status */
    *delivery = data[8];
    
    /* Done */
    return XBEE_RET_OK;
}


/*!
 * Get the extended transmit status.
 *
 * @param[out]  addr        16-bit address received
 * @param[out]  retries     Number of transmission retries received
 * @param[out]  delivery    Delivery status received
 * @param[out]  discovery   Discovery status received
 * @param[out]  fid         Frame ID received
 * @return      OK in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_transmit_status_ext(uint16_t* addr, uint8_t* retries, uint8_t* delivery, uint8_t* discovery, uint8_t* fid) {
    /* Data array for the frame (max. length: 23 bytes) */
    uint8_t data[23] = {0};
    /* Length of the frame to be received */
    uint16_t len = 0;
    /* Reading position (index) */
    uint8_t pos = 0;
    /* Flag for completed response */
    uint8_t complete = 0;
    
    /*** Read data ***/
    uint16_t timeout = XBEE_RESPONSE_TIMEOUT * (1000UL / XBEE_RESPONSE_TIMEOUT_DELAY);
    do {
        /* Check if data is available to be received */
        if(_available()) {
            /* Read next byte */
            if(_read(&data[pos]) != 0) {
                /* Read failed */
                continue;
            }
            /* Store/process data based on frame position (index) */
            switch(pos) {
                /* First byte should be the start delimiter */
                case 0:
                    /* Check if current byte is the start delimiter */
                    if(data[pos] == XBEE_START_DELIMITER) {
                        /* Increment position */
                        pos++;
                    }
                    /* Store value */
                    break;
                /* Second byte should be the length MSB */
                case 1:
                    /* Add MSB value to len variable */
                    len |= (uint16_t)data[pos] << 8;
                    /* Increment position */
                    pos++;
                    
                    break;
                /* Third byte should be the length LSB */
                case 2:
                    /* Add LSB value to len variable */
                    len |= (uint16_t)data[pos];
                    /* Increment position */
                    pos++;
                    
                    break;
                /* All other bytes ... */
                default:
                    /* Check if maximum frame data size has been exceeded */
                    if(pos > (23-1)) {
                        /* Size limit exceed */
                        return XBEE_RET_PAYLOAD_SIZE_EXCEEDED;
                    }
                    /* Check if the end of the frame has been reached */
                    /* Packet length does not include start, length, or checksum bytes -> add 3 */
                    if(pos == (len+3)) {
                        /* Reception done ... continue with data processing */
                        complete = 1;
                        break;
                    } else {
                        /* Increment position */
                        pos++;
                    }
                    
                    break;
            }
        }
        /* Wait for some time */
        _delay_ms(XBEE_RESPONSE_TIMEOUT_DELAY);
    } while((--timeout) && (complete==0));
    /* Check if timeout has triggered */
    if(timeout == 0) {
        /* Response timed out */
        return XBEE_RET_TIMEOUT;
    }
    
    /*** Process data ***/
    /* Check the frame type */
    if(data[3] != XBEE_FRAME_TRANSMIT_STATUS_EXT) {
        /* Frame type does not match */
        return XBEE_RET_INVALID_FRAME;
    }
    /* Check the CRC */
    if(xbee_check_crc(&data[3], 7, data[10]) != XBEE_RET_OK) {
        return XBEE_RET_INVALID_CRC;
    }
    /* Get the frame id */
    *fid = data[4];
    /* Get the sender's 16-bit address */
    *addr = ((uint16_t)data[5] << 8) | (uint16_t)data[6];
    /* Get the Transmit retry count */
    *retries = data[7];
    /* Get the Delivery status */
    *delivery = data[8];
    /* Get the Discovery status */
    *discovery = data[9];
    
    /* Done */
    return XBEE_RET_OK;
}


/*!
 * Get the CRC value of a given frame.
 *
 * @param[in]   data        Frame data
 * @param[in]   len         Number of bytes in frame.
 * @return      Calculated CRC
 */
uint8_t xbee_get_crc(uint8_t* data, uint8_t len) {
    uint16_t checksum = 0;
    uint8_t i;
    
    /* Iterate over the data bytes (ignore first three bytes) */
    for(i=0; i<len; i++) {
        checksum = checksum + data[i];
    }
    /* Subtract checksum from 0xFF */
    checksum = 0xFF - (checksum & 0xFF);
    /* Return the calculated CRC value */
    return (uint8_t)checksum;
}


/*!
 * Check the CRC value of a frame.
 *
 * @param[in]   data        Frame data
 * @param[in]   len         Number of bytes in frame.
 * @param[in]   crc         Received CRC.
 * @return      OK in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_check_crc(uint8_t* data, uint8_t len, uint8_t crc) {
    uint16_t checksum = 0;
    /* Iterate over the data bytes (ignore first three bytes) */
    for(uint8_t i=0; i<len; i++) {
        checksum = checksum + data[i];
    }
    /* Subtract checksum from 0xFF */
    checksum = 0xFF - checksum;
    /* Check if calculated CRC matches received CRC */
    if((uint8_t)checksum == crc) {
        /* Checksum is correct */
        return XBEE_RET_OK;
    } else {
        /* Checksum is incorrect */
        return XBEE_RET_INVALID_CRC;
    }
}


/*!
 * Transmit a specified number of bytes via broadcast.
 *
 * @param[in]   payload     Payload data to be transmitted
 * @param[in]   cnt         Number of payload bytes
 * @param[in]   fid         Frame ID.
 * @return      OK in case of success; ERROR otherwise
 */
inline XBEE_RET_t xbee_transmit_broadcast(uint8_t* payload, uint16_t cnt, uint8_t fid) {
    return xbee_transmit(XBEE_ADDR64_BROADCAST, XBEE_ADDR16_ALL_DEVICES, payload, cnt, fid);
}


/*!
 * Transmit a specified number of bytes via unicast to a specific destination.
 *
 * @param[in]   mac         MAC address of the destination
 * @param[in]   payload     Payload data to be transmitted
 * @param[in]   cnt         Number of payload bytes
 * @param[in]   fid         Frame ID.
 * @return      OK in case of success; ERROR otherwise
 */
inline XBEE_RET_t xbee_transmit_unicast(uint64_t mac, uint8_t* payload, uint16_t cnt, uint8_t fid) {
    return xbee_transmit(mac, XBEE_ADDR16_USE64, payload, cnt, fid);
}


/*!
 * Check if the device has successfully joined a network.
 *
 * @return      OK in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_is_connected(void) {
    uint64_t response;
    /* Check Xbee connection status (need 1 return byte) */
    if(xbee_at_local_cmd_read((char *)"AI", &response) == 1) {
        if(response == XBEE_AI_RET_SUCCESS) {
            /* Successfully connected */
            return XBEE_RET_OK;
        }
    }
    /* Not connected */
    return XBEE_RET_ERROR;
}


/*!
 * Wait until the device has successfully joined a network.
 *
 * @return      OK in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_wait_for_connected(uint8_t timeout) {
    /* Get maximum number of retries */
    uint16_t retries = timeout * (1000UL / XBEE_JOIN_TIMEOUT_DELAY);
    /* Check xbee's response */
    while(retries--) {
        /* Check Xbee module connection */
        if(xbee_is_connected() == XBEE_RET_OK) {
            /* Connection established successfully */
            return XBEE_RET_OK;
        }
        /* Wait for some time */
        _delay_ms(XBEE_JOIN_TIMEOUT_DELAY);
    }
    /* Connection established failed */
    return XBEE_RET_TIMEOUT;
}


/*!
 * Read the current device temperature.
 *
 * @param[out]  temp        Device temperature read
 * @return      OK in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_cmd_get_temperature(float* temp) {
    uint64_t retval;
    int8_t ret = xbee_at_local_cmd_read((char *)"TP", &retval);
    if(ret != 2) {
        /* Return error */
        return ret;
    }
    /* Copy temperature reading */
    *temp = (float)(retval & 0xFFFF);
    /* Seem like everything worked */
    return XBEE_RET_OK;
}


/*!
 * Read the current supply voltage level.
 *
 * @param[out]  vss         Device supply voltage level
 * @return      OK in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_cmd_get_vss(float* vss) {
    uint64_t retval;
    int8_t ret = xbee_at_local_cmd_read((char *)"%V", &retval);
    if(ret != 2) {
        /* Return error */
        return ret;
    }
    /* Copy temperature reading */
    *vss = ((float)retval / 1000.0);
    /* Seem like everything worked */
    return XBEE_RET_OK;
}
