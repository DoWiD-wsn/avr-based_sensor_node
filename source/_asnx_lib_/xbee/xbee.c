/*!
 * @brief   ASN(x) Xbee 3 library -- source file
 *
 * Library to the Xbee 3 module accessible via UART.
 *
 * @file    /_asnx_lib_/xbee/xbee.c
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 *
 * @todo    *) Implement better way for (asynchronous) response handling/matching
 * @todo    *) Fix blocking (non-ISR) functions (not really possible now)
 * @todo    *) Check how long it takes for the Xbee to (re)join a network after waking up
 * @todo    *) Check if the local "AI" command actually returns the right value or if there are issues (cross-check with BLE app)
 */


/***** INCLUDES *******************************************************/
#include "xbee.h"


/***** GLOBAL VARIABLES ***********************************************/
/*! Xbee sleep request pin GPIO structure */
hw_io_t xbee_sleep_req;
/*! Xbee sleep indicator pin GPIO structure */
hw_io_t xbee_sleep_ind;


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
 * @param[in]   baud        BAUD rate to be used.
 */
void xbee_init(uint32_t baud) {
    /* Initialize the UART interface */
    uart0_init();
    uart0_set_baudrate(baud);
    uart0_interrupt_enable();
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
    uint32_t retries = 0;
    while(hw_read_input(&xbee_sleep_ind) == HW_STATE_HIGH) {
        /* Check if timeout [s] has been reached (counter in [ms]) */
        if(retries >= ((uint32_t)XBEE_WAKE_TIMEOUT*1000)) {
            /* Couldn't send xbee to sleep */
            return XBEE_RET_ERROR;
        } else {
            /* Wait for some time */
            retries += XBEE_WAKE_TIMEOUT_DELAY;
            _delay_ms(XBEE_WAKE_TIMEOUT_DELAY);
        }
    }
    /* Sleep request successful */
    return XBEE_RET_OK;
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
    uint32_t retries = 0;
    while(hw_read_input(&xbee_sleep_ind) == HW_STATE_LOW) {
        /* Check if timeout [s] has been reached (counter in [ms]) */
        if(retries >= ((uint32_t)XBEE_WAKE_TIMEOUT*1000)) {
            /* Couldn't wake xbee up */
            return XBEE_RET_ERROR;
        } else {
            /* Wait for some time */
            retries += XBEE_WAKE_TIMEOUT_DELAY;
            _delay_ms(XBEE_WAKE_TIMEOUT_DELAY);
        }
    }
    /* Wake-up successful */
    return XBEE_RET_OK;
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
            data[7] = (value >> 56) & 0xFF;
            data[8] = (value >> 48) & 0xFF;
            data[9] = (value >> 40) & 0xFF;
            data[10] = (value >> 32) & 0xFF;
            data[11] = (value >> 24) & 0xFF;
            data[12] = (value >> 16) & 0xFF;
            data[13] = (value >> 8) & 0xFF;
            data[14] = value & 0xFF;
            break;
        case 8:
            /* 32-bit value */
            data[7] = (value >> 24) & 0xFF;
            data[8] = (value >> 16) & 0xFF;
            data[9] = (value >> 8) & 0xFF;
            data[10] = value & 0xFF;
            break;
        case 6:
            /* 16-bit value */
            data[7] = (value >> 8) & 0xFF;
            data[8] = value & 0xFF;
            break;
        case 5:
            /* 8-bit value */
            data[7] = value & 0xFF;
            break;
        default:
            /* Should never happen */
            return XBEE_RET_ERROR;
    }
    /* Last byte is the CRC (ignore first 3 bytes) */
    data[len+3] = xbee_get_crc(&data[3], len);
    
#if XBEE_WRITE_NONBLOCKING==0
    if(uart0_write_blocking(data, (len+4)) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#else
    if(uart0_write(data, (len+4)) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#endif
    
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
    
    /* Send the complete frame via UART */
#if XBEE_WRITE_NONBLOCKING==0
    if(uart0_write_blocking(data, 8) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#else
    if(uart0_write(data, 8) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#endif
    
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
    /* Temporary variables for frame length (depends on value) and timeout */
    uint16_t len, diff;
    uint32_t timeout=0;
    /* Check if first byte in RX buffer is the Frame Delimiter */
    do {
        /* Check if a byte has been received */
        while(uart0_read(&data[0], 1) != 1) {
            /* Check if timeout has been reached */
            if(timeout >= ((uint32_t)XBEE_RX_TIMEOUT*1000)) {
                /* No frame was received */
                return XBEE_RET_ERROR;
            }
            /* Wait for some time */
            _delay_ms(XBEE_RX_TIMEOUT_DELAY);
            /* Increment timeout counter */
            timeout += XBEE_RX_TIMEOUT_DELAY;
        }
    /* Do until the Start Delimiter was received (or timeout has been reached) */
    }while(data[0] != XBEE_START_DELIMITER);
    /* Reset timeout */
    timeout=0;
    /* Check if 7 more bytes are available already (incl. the length and cmd status) */
    while(uart0_rx_buffer_cnt() < 7) {
        /* Check if timeout has been reached */
        if(timeout >= ((uint32_t)XBEE_RX_TIMEOUT*1000)) {
            /* No frame was received */
            return XBEE_RET_ERROR;
        }
        /* Wait for some time */
        _delay_ms(XBEE_RX_TIMEOUT_DELAY);
        /* Increment timeout counter */
        timeout += XBEE_RX_TIMEOUT_DELAY;
    }
    /* Reset timeout */
    timeout=0;
    /* Read the 7 bytes */
    if(uart0_read(&data[1], 7) != 7) {
        /* There should be at least 7 bytes available ... */
        return XBEE_RET_ERROR;
    }
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
    /* If we came so far, everything seems to be fine ... check length field */
    len = ((uint16_t)data[1] << 8) | (uint16_t)data[2];
    /* Check how many more bytes need to be read */
    diff = len - 5;
    /* Check if "diff" (+1 for crc) more bytes are available */
    while(uart0_rx_buffer_cnt() < (diff+1)) {
        /* Check if timeout has been reached */
        if(timeout >= ((uint32_t)XBEE_RX_TIMEOUT*1000)) {
            /* No frame was received */
            return XBEE_RET_ERROR;
        }
        /* Wait for some time */
        _delay_ms(XBEE_RX_TIMEOUT_DELAY);
        /* Increment timeout counter */
        timeout += XBEE_RX_TIMEOUT_DELAY;
    }
    /* Read the "diff" (+1) bytes */
    if(uart0_read(&data[8], (diff+1)) != (diff+1)) {
        /* There should be at least (diff+1) bytes available ... */
        return XBEE_RET_ERROR;
    }
    /* Remained depends on command data size */
    switch(diff) {
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
 * @param[in]   fid         Frame ID.
 * @return      Size of the command value in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_at_local_cmd_write(char* command, uint64_t value, uint8_t fid){
    int8_t ret;
    uint8_t fid_ret;
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
 * @param[in]   fid         Frame ID.
 * @return      Size of the command value in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_at_local_cmd_read(char* command, uint64_t* value, uint8_t fid) {
    int8_t ret;
    uint8_t fid_ret;
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
    uint8_t i=0;
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
    
#if XBEE_WRITE_NONBLOCKING==0
    if(uart0_write_blocking(data, (len+4)) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#else
    if(uart0_write(data, (len+4)) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#endif
    
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
    
#if XBEE_WRITE_NONBLOCKING==0
    if(uart0_write_blocking(data, 19) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#else
    if(uart0_write(data, 19) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#endif
    
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
    uint8_t i=0;
    /* Data array for the frame (max. length: 27 bytes) */
    uint8_t data[27] = {0};
    /* Number of bytes of the response value received */
    uint8_t resp_cnt = 0;
    /* Temporary variables for frame length (depends on value) and timeout */
    uint16_t len, diff;
    uint32_t timeout=0;
    /* Check if first byte in RX buffer is the Frame Delimiter */
    do {
        /* Check if a byte has been received */
        while(uart0_read(&data[0], 1) != 1) {
            /* Check if timeout has been reached */
            if(timeout >= ((uint32_t)XBEE_RX_TIMEOUT*1000)) {
                /* No frame was received */
                return XBEE_RET_ERROR;
            }
            /* Wait for some time */
            _delay_ms(XBEE_RX_TIMEOUT_DELAY);
            /* Increment timeout counter */
            timeout += XBEE_RX_TIMEOUT_DELAY;
        }
    /* Do until the Start Delimiter was received (or timeout has been reached) */
    }while(data[0] != XBEE_START_DELIMITER);
    /* Reset timeout */
    timeout=0;
    /* Check if 17 more bytes are available already (incl. the length and cmd status) */
    while(uart0_rx_buffer_cnt() < 17) {
        /* Check if timeout has been reached */
        if(timeout >= ((uint32_t)XBEE_RX_TIMEOUT*1000)) {
            /* No frame was received */
            return XBEE_RET_ERROR;
        }
        /* Wait for some time */
        _delay_ms(XBEE_RX_TIMEOUT_DELAY);
        /* Increment timeout counter */
        timeout += XBEE_RX_TIMEOUT_DELAY;
    }
    /* Reset timeout */
    timeout=0;
    /* Read the 17 bytes */
    if(uart0_read(&data[1], 17) != 17) {
        /* There should be at least 17 bytes available ... */
        return XBEE_RET_ERROR;
    }
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
    /* If we came so far, everything seems to be fine ... check length field */
    len = ((uint16_t)data[1] << 8) | (uint16_t)data[2];
    /* Check how many more bytes need to be read */
    diff = len - 15;
    /* Check if "diff" (+1 for crc) more bytes are available */
    while(uart0_rx_buffer_cnt() < (diff+1)) {
        /* Check if timeout has been reached */
        if(timeout >= ((uint32_t)XBEE_RX_TIMEOUT*1000)) {
            /* No frame was received */
            return XBEE_RET_ERROR;
        }
        /* Wait for some time */
        _delay_ms(XBEE_RX_TIMEOUT_DELAY);
        /* Increment timeout counter */
        timeout += XBEE_RX_TIMEOUT_DELAY;
    }
    /* Read the "diff" (+1) bytes */
    if(uart0_read(&data[18], (diff+1)) != (diff+1)) {
        /* There should be at least (diff+1) bytes available ... */
        return XBEE_RET_ERROR;
    }
    /* Remained depends on command data size */
    switch(diff) {
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
 * @param[in]   fid         Frame ID.
 * @return      Size of the command value in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_at_remote_cmd_write(uint64_t mac, uint16_t addr, char* command, uint64_t value, uint8_t fid) {
    uint8_t fid_ret;
    uint16_t addr_ret;
    uint64_t mac_ret;
    int8_t ret;
    /* There should be no response value, still ... */
    uint64_t resp;
    
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
 * @param[in]   fid         Frame ID.
 * @return      Size of the command value in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_at_remote_cmd_read(uint64_t mac, uint16_t addr, char* command, uint64_t* value, uint8_t fid) {
    uint8_t fid_ret;
    uint16_t addr_ret;
    uint64_t mac_ret;
    int8_t ret;
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
    
#if XBEE_WRITE_NONBLOCKING==0
    if(uart0_write_blocking(data, (len+4)) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#else
    if(uart0_write(data, (len+4)) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#endif
    
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
    /* Temporary variables for frame length (depends on value) and timeout */
    uint32_t timeout=0;
    /* Check if first byte in RX buffer is the Frame Delimiter */
    do {
        /* Check if a byte has been received */
        while(uart0_read(&data[0], 1) != 1) {
            /* Check if timeout has been reached */
            if(timeout >= ((uint32_t)XBEE_RX_TIMEOUT*1000)) {
                /* No frame was received */
                return XBEE_RET_TIMEOUT;
            }
            /* Wait for some time */
            _delay_ms(XBEE_RX_TIMEOUT_DELAY);
            /* Increment timeout counter */
            timeout += XBEE_RX_TIMEOUT_DELAY;
        }
    /* Do until the Start Delimiter was received (or timeout has been reached) */
    }while(data[0] != XBEE_START_DELIMITER);
    /* Reset timeout */
    timeout=0;
    /* Check if the remaining 10 bytes are available already */
    while(uart0_rx_buffer_cnt() < 10) {
        /* Check if timeout has been reached */
        if(timeout >= ((uint32_t)XBEE_RX_TIMEOUT*1000)) {
            /* No frame was received */
            return XBEE_RET_TIMEOUT;
        }
        /* Wait for some time */
        _delay_ms(XBEE_RX_TIMEOUT_DELAY);
        /* Increment timeout counter */
        timeout += XBEE_RX_TIMEOUT_DELAY;
    }
    /* Reset timeout */
    timeout=0;
    /* Read the 10 bytes */
    if(uart0_read(&data[1], 10) != 10) {
        /* There should be at least 10 bytes available ... */
        return XBEE_RET_ERROR;
    }
    /* Check the frame type */
    if(data[3] != XBEE_FRAME_TRANSMIT_STATUS_EXT) {
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
    uint8_t data[11] = {0};
    /* Temporary variables for frame length (depends on value) and timeout */
    uint32_t timeout=0;
    /* Check if first byte in RX buffer is the Frame Delimiter */
    do {
        /* Check if a byte has been received */
        while(uart0_read(&data[0], 1) != 1) {
            /* Check if timeout has been reached */
            if(timeout >= ((uint32_t)XBEE_RX_TIMEOUT*1000)) {
                /* No frame was received */
                return XBEE_RET_ERROR;
            }
            /* Wait for some time */
            _delay_ms(XBEE_RX_TIMEOUT_DELAY);
            /* Increment timeout counter */
            timeout += XBEE_RX_TIMEOUT_DELAY;
        }
    /* Do until the Start Delimiter was received (or timeout has been reached) */
    }while(data[0] != XBEE_START_DELIMITER);
    /* Reset timeout */
    timeout=0;
    /* Check if the remaining 10 bytes are available already */
    while(uart0_rx_buffer_cnt() < 10) {
        /* Check if timeout has been reached */
        if(timeout >= ((uint32_t)XBEE_RX_TIMEOUT*1000)) {
            /* No frame was received */
            return XBEE_RET_ERROR;
        }
        /* Wait for some time */
        _delay_ms(XBEE_RX_TIMEOUT_DELAY);
        /* Increment timeout counter */
        timeout += XBEE_RX_TIMEOUT_DELAY;
    }
    /* Reset timeout */
    timeout=0;
    /* Read the 10 bytes */
    if(uart0_read(&data[1], 10) != 10) {
        /* There should be at least 10 bytes available ... */
        return XBEE_RET_ERROR;
    }
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
    checksum = 0xFF - checksum;
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
    uint8_t i;
    /* Iterate over the data bytes (ignore first three bytes) */
    for(i=0; i<len; i++) {
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
    if(xbee_at_local_cmd_read("AI", &response, 0x01) == 1) {
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
    uint32_t time = 0;
    /* Check Xbee module connection */
    while(xbee_is_connected() != XBEE_RET_OK) {
        /* Check if timeout [s] has been reached (counter in [ms]) */
        if(time >= (timeout*1000)) {
            return XBEE_RET_ERROR;
        } else {
            /* Wait for some time */
            time += XBEE_JOIN_TIMEOUT_DELAY;
            _delay_ms(XBEE_JOIN_TIMEOUT_DELAY);
        }
    }
    return XBEE_RET_OK;
}


/*!
 * Read the current device temperature.
 *
 * @param[out]  temp        Device temperature read
 * @return      OK in case of success; ERROR otherwise
 */
XBEE_RET_t xbee_cmd_get_temperature(float* temp) {
    int8_t ret;
    uint64_t retval;
    ret = xbee_at_local_cmd_read("TP", &retval, 0x01);
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
    int8_t ret;
    uint64_t retval;
    ret = xbee_at_local_cmd_read("%V", &retval, 0x01);
    if(ret != 2) {
        /* Return error */
        return ret;
    }
    /* Copy temperature reading */
    *vss = ((float)retval / 1000.0);
    /* Seem like everything worked */
    return XBEE_RET_OK;
}


/*!
 * Flush the receive buffer.
 */
void xbee_rx_flush(void) {
    /* Flush the UART RX buffer */
    uart0_rx_flush();
}


/*!
 * Flush the transmit buffer.
 */
void xbee_tx_flush(void) {
    /* Flush the UART TX buffer */
    uart0_tx_flush();
}


/*!
 * Get the number of bytes in the RX buffer.
 *
 * @return      Number of bytes in the RX buffer.
 */
uint8_t xbee_rx_cnt(void) {
    return uart0_rx_buffer_cnt();
}


/*!
 * Get the number of bytes in the TX buffer.
 *
 * @return      Number of bytes in the TX buffer.
 */
uint8_t xbee_tx_cnt(void) {
    return uart0_tx_buffer_cnt();
}
