/**
 *  Source file for Xbee 3 functionality.
 */

/***** INCLUDES ***************************************************************/
/* STD */
/* AVR */
#include <util/delay.h>
/* OWN */
#include "xbee/xbee.h"
#include "uart/uart.h"


/***** MACROS *****************************************************************/


/***** ENUMERATION ************************************************************/


/***** STRUCTURES *************************************************************/


/***** GLOBAL VARIABLES *******************************************************/


/***** LOCAL FUNCTION PROTOTYPES **********************************************/
/* Local AT command */
int8_t xbee_at_local_write(char* command, uint64_t value, uint8_t fid);
int8_t xbee_at_local_query(char* command, uint8_t fid);
int8_t xbee_at_local_response(uint64_t* value, uint8_t* fid);
/* Remote AT command */
int8_t xbee_at_remote_write(uint64_t mac, uint16_t addr, char* command, uint64_t value, uint8_t fid);
int8_t xbee_at_remote_query(uint64_t mac, uint16_t addr, char* command, uint8_t fid);
int8_t xbee_at_remote_response(uint64_t* mac, uint16_t* addr, uint64_t* value, uint8_t* fid);


/***** WRAPPER FUNCTIONS ******************************************************/


/***** FUNCTIONS **************************************************************/
/*** COMMON ***************************/
/*
 * Initialize everything needed for Xbee communication (e.g., UART)
 */
void xbee_init(uint32_t baud) {
    /* Initialize the UART interface */
    uart_init();
    uart_set_baudrate(baud);
    uart_interrupt_enable();
}


/*** SENDING/RECEIVING COMMANDS *******/

/*
 *  Write a local AT command
 */
int8_t xbee_at_local_write(char* command, uint64_t value, uint8_t fid) {
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
    if(uart_write_blocking(data, (len+4)) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#else
    if(uart_write(data, (len+4)) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#endif
    
    /* Done */
    return XBEE_RET_OK;
}


/*
 *  Query a local AT command
 */
int8_t xbee_at_local_query(char* command, uint8_t fid) {
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
    if(uart_write_blocking(data, 8) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#else
    if(uart_write(data, 8) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#endif
    
    /* Done */
    return XBEE_RET_OK;
}


/*
 * Get the response to a local AT command
 */
int8_t xbee_at_local_response(uint64_t* value, uint8_t* fid) {
    /* Data array for the frame (max. length: 17 bytes) */
    uint8_t data[17] = {0};
    /* Number of bytes of the response value received */
    uint8_t resp_cnt = 0;
    /* Temporary variables for frame length (depends on value) and timeout */
    uint16_t len, diff, timeout=0;
    /* Check if first byte in RX buffer is the Frame Delimiter */
    do {
        /* Check if a byte has been received */
        while(uart_read(&data[0], 1) != 1) {
            /* Check if timeout has been reached */
            if(timeout >= XBEE_RX_TIMEOUT) {
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
    while(uart_rx_buffer_cnt() < 7) {
        /* Check if timeout has been reached */
        if(timeout >= XBEE_RX_TIMEOUT) {
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
    if(uart_read(&data[1], 7) != 7) {
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
    while(uart_rx_buffer_cnt() < (diff+1)) {
        /* Check if timeout has been reached */
        if(timeout >= XBEE_RX_TIMEOUT) {
            /* No frame was received */
            return XBEE_RET_ERROR;
        }
        /* Wait for some time */
        _delay_ms(XBEE_RX_TIMEOUT_DELAY);
        /* Increment timeout counter */
        timeout += XBEE_RX_TIMEOUT_DELAY;
    }
    /* Read the "diff" (+1) bytes */
    if(uart_read(&data[8], (diff+1)) != (diff+1)) {
        /* There should be at least (diff+1) bytes available ... */
        return XBEE_RET_ERROR;
    }
    /* Remained depends on command data size */
    switch(diff) {
        case 0:         // No command data
            /* No command data -> check CRC */
            if(!xbee_check_crc(&data[3], 5, data[8])) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 0 bytes value */
            resp_cnt = 0;
            break;
        case 1:         // 8-bit command data
            /* Check the CRC */
            if(!xbee_check_crc(&data[3], 6, data[9])) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 1 command data byte */
            *value = data[8];
            resp_cnt = 1;
            break;
        case 2:         // 16-bit command data
            /* Check the CRC */
            if(!xbee_check_crc(&data[3], 7, data[10])) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 2 command data byte */
            *value = ((uint16_t)data[8]<<8);
            *value |= (uint16_t)data[9];
            resp_cnt = 2;
            break;
        case 3:         // 24-bit command data
            /* Check the CRC */
            if(!xbee_check_crc(&data[3], 8, data[11])) {
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
            if(!xbee_check_crc(&data[3], 9, data[12])) {
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
            if(!xbee_check_crc(&data[3], 10, data[13])) {
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
            if(!xbee_check_crc(&data[3], 11, data[14])) {
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
            if(!xbee_check_crc(&data[3], 12, data[15])) {
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
            if(!xbee_check_crc(&data[3], 13, data[16])) {
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


/*
 * Write a local AT command and check the response
 */
int8_t xbee_at_local_cmd_write(char* command, uint64_t value, uint8_t fid){
    int8_t ret;
    uint8_t fid_ret;
    /* There should be no response value, still ... */
    uint64_t resp;
    
    /* Send the local AT command with the new value */
    ret = xbee_at_local_write(command, value, fid);
    if(ret < XBEE_RET_OK) {
        /* Sending failed */
        return ret;
    }
    
    /* Check the response */
    ret = xbee_at_local_response(&resp, &fid_ret);
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


/*
 * Read the response to a local AT command
 */
int8_t xbee_at_local_cmd_read(char* command, uint64_t* value, uint8_t fid) {
    int8_t ret;
    uint8_t fid_ret;
    /* Send the local AT command */
    ret = xbee_at_local_query(command, fid);
    if(ret != XBEE_RET_OK) {
        /* Sending failed */
        return ret;
    }
    
    /* Check the response */
    ret = xbee_at_local_response(value, &fid_ret);
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


/*
 *  Write a remote AT command.
 *  Remote target specified by MAC or Address
 */
int8_t xbee_at_remote_write(uint64_t mac, uint16_t addr, char* command, uint64_t value, uint8_t fid) {
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
    if(uart_write_blocking(data, (len+4)) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#else
    if(uart_write(data, (len+4)) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#endif
    
    /* Done */
    return XBEE_RET_OK;
}


/*
 *  Query a remote AT command
 *  Remote target specified by MAC or Address
 */
int8_t xbee_at_remote_query(uint64_t mac, uint16_t addr, char* command, uint8_t fid) {
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
    if(uart_write_blocking(data, 19) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#else
    if(uart_write(data, 19) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#endif
    
    /* Done */
    return XBEE_RET_OK;
}


/*
 *  Get the response to a remote AT command
 */
int8_t xbee_at_remote_response(uint64_t* mac, uint16_t* addr, uint64_t* value, uint8_t* fid) {
    uint8_t i=0;
    /* Data array for the frame (max. length: 27 bytes) */
    uint8_t data[27] = {0};
    /* Number of bytes of the response value received */
    uint8_t resp_cnt = 0;
    /* Temporary variables for frame length (depends on value) and timeout */
    uint16_t len, diff, timeout=0;
    /* Check if first byte in RX buffer is the Frame Delimiter */
    do {
        /* Check if a byte has been received */
        while(uart_read(&data[0], 1) != 1) {
            /* Check if timeout has been reached */
            if(timeout >= XBEE_RX_TIMEOUT) {
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
    while(uart_rx_buffer_cnt() < 17) {
        /* Check if timeout has been reached */
        if(timeout >= XBEE_RX_TIMEOUT) {
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
    if(uart_read(&data[1], 17) != 17) {
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
    while(uart_rx_buffer_cnt() < (diff+1)) {
        /* Check if timeout has been reached */
        if(timeout >= XBEE_RX_TIMEOUT) {
            /* No frame was received */
            return XBEE_RET_ERROR;
        }
        /* Wait for some time */
        _delay_ms(XBEE_RX_TIMEOUT_DELAY);
        /* Increment timeout counter */
        timeout += XBEE_RX_TIMEOUT_DELAY;
    }
    /* Read the "diff" (+1) bytes */
    if(uart_read(&data[18], (diff+1)) != (diff+1)) {
        /* There should be at least (diff+1) bytes available ... */
        return XBEE_RET_ERROR;
    }
    /* Remained depends on command data size */
    switch(diff) {
        case 0:         // No command data
            /* No command data -> check CRC */
            if(!xbee_check_crc(&data[3], 15, data[18])) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 0 command data bytes */
            resp_cnt = 0;
            break;
        case 1:         // 8-bit command data
            /* Check the CRC */
            if(!xbee_check_crc(&data[3], 16, data[19])) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 1 command data byte */
            *value = data[18];
            resp_cnt = 1;
            break;
        case 2:         // 16-bit command data
            /* Check the CRC */
            if(!xbee_check_crc(&data[3], 17, data[20])) {
                return XBEE_RET_INVALID_CRC;
            }
            /* 2 command data byte */
            *value = ((uint16_t)data[18]<<8);
            *value |= (uint16_t)data[19];
            resp_cnt = 2;
            break;
        case 3:         // 24-bit command data
            /* Check the CRC */
            if(!xbee_check_crc(&data[3], 18, data[21])) {
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
            if(!xbee_check_crc(&data[3], 19, data[22])) {
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
            if(!xbee_check_crc(&data[3], 20, data[23])) {
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
            if(!xbee_check_crc(&data[3], 21, data[24])) {
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
            if(!xbee_check_crc(&data[3], 22, data[25])) {
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
            if(!xbee_check_crc(&data[3], 23, data[26])) {
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


/*
 * Write a remote AT command and check the response
 */
int8_t xbee_at_remote_cmd_write(uint64_t mac, uint16_t addr, char* command, uint64_t value, uint8_t fid) {
    uint8_t fid_ret;
    uint16_t addr_ret;
    uint64_t mac_ret;
    int8_t ret;
    /* There should be no response value, still ... */
    uint64_t resp;
    
    /* Send the remote AT command */
    ret = xbee_at_remote_write(mac, addr, command, value, fid);
    if(ret < XBEE_RET_OK) {
        /* Sending failed */
        return ret;
    }
    
    /* Check the response */
    ret = xbee_at_remote_response(&mac_ret, &addr_ret, &resp, &fid_ret);
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


/*
 * Read the response to a remote AT command
 */
int8_t xbee_at_remote_cmd_read(uint64_t mac, uint16_t addr, char* command, uint64_t* value, uint8_t fid) {
    uint8_t fid_ret;
    uint16_t addr_ret;
    uint64_t mac_ret;
    int8_t ret;
    /* Send the remote AT command */
    ret = xbee_at_remote_query(mac, addr, command, fid);
    if(ret < XBEE_RET_OK) {
        /* Sending failed */
        return ret;
    }
    
    /* Check the response */
    ret = xbee_at_remote_response(&mac_ret, &addr_ret, value, &fid_ret);
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


/*
 *  Transmit a specified number of bytes (len) to a destination
 *  Destination target specified by MAC or Address
 */
int8_t xbee_transmit(uint64_t mac, uint16_t addr, uint8_t* payload, uint16_t cnt, uint8_t fid) {
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
    if(uart_write_blocking(data, (len+4)) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#else
    if(uart_write(data, (len+4)) != UART_RET_OK) {
        /* UART write failed */
        return XBEE_RET_ERROR;
    }
#endif
    
    /* Done */
    return XBEE_RET_OK;
}


/*
 *  Get the transmit status
 */
int8_t xbee_transmit_status(uint8_t* delivery) {
    /* Data array for the frame (max. length: 23 bytes) */
    uint8_t data[11] = {0};
    /* Temporary variables for frame length (depends on value) and timeout */
    uint16_t timeout=0;
    /* Check if first byte in RX buffer is the Frame Delimiter */
    do {
        /* Check if a byte has been received */
        while(uart_read(&data[0], 1) != 1) {
            /* Check if timeout has been reached */
            if(timeout >= XBEE_RX_TIMEOUT) {
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
    while(uart_rx_buffer_cnt() < 10) {
        /* Check if timeout has been reached */
        if(timeout >= XBEE_RX_TIMEOUT) {
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
    if(uart_read(&data[1], 10) != 10) {
        /* There should be at least 10 bytes available ... */
        return XBEE_RET_ERROR;
    }
    /* Check the frame type */
    if(data[3] != XBEE_FRAME_TRANSMIT_STATUS_EXT) {
        /* Frame type does not match */
        return XBEE_RET_INVALID_FRAME;
    }
    /* Check the CRC */
    if(!xbee_check_crc(&data[3], 7, data[10])) {
        return XBEE_RET_INVALID_CRC;
    }
    /* Get the Delivery status */
    *delivery = data[8];
    
    /* Done */
    return XBEE_RET_OK;
}


/*
 *  Get the extended transmit status
 */
int8_t xbee_transmit_status_ext(uint16_t* addr, uint8_t* retries, uint8_t* delivery, uint8_t* discovery, uint8_t* fid) {
    /* Data array for the frame (max. length: 23 bytes) */
    uint8_t data[11] = {0};
    /* Temporary variables for frame length (depends on value) and timeout */
    uint16_t timeout=0;
    /* Check if first byte in RX buffer is the Frame Delimiter */
    do {
        /* Check if a byte has been received */
        while(uart_read(&data[0], 1) != 1) {
            /* Check if timeout has been reached */
            if(timeout >= XBEE_RX_TIMEOUT) {
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
    while(uart_rx_buffer_cnt() < 10) {
        /* Check if timeout has been reached */
        if(timeout >= XBEE_RX_TIMEOUT) {
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
    if(uart_read(&data[1], 10) != 10) {
        /* There should be at least 10 bytes available ... */
        return XBEE_RET_ERROR;
    }
    /* Check the frame type */
    if(data[3] != XBEE_FRAME_TRANSMIT_STATUS_EXT) {
        /* Frame type does not match */
        return XBEE_RET_INVALID_FRAME;
    }
    /* Check the CRC */
    if(!xbee_check_crc(&data[3], 7, data[10])) {
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


/*** CRC Check ************************/

/*
 * Get the CRC value of a frame
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


/*
 * Check the CRC value of a frame
 */
uint8_t xbee_check_crc(uint8_t* data, uint8_t len, uint8_t crc) {
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
        return 1;
    } else {
        /* Checksum is incorrect */
        return 0;
    }
}


/**************************************/
/*** COMMON FUNCTIONALITY *************/
/**************************************/

/*
 * Read the current device temperature
 */
int8_t xbee_cmd_get_temperature(int16_t* temp) {
    int8_t ret;
    uint64_t retval;
    ret = xbee_at_local_cmd_read("TP", &retval, 0x01);
    if(ret != 2) {
        /* Return error */
        return ret;
    }
    /* Copy temperature reading */
    *temp = (int16_t)(retval & 0xFFFF);
    /* Seem like everything worked */
    return XBEE_RET_OK;
}


/*
 * Read the current supply voltage level
 */
int8_t xbee_cmd_get_vss(uint16_t* vss) {
    int8_t ret;
    uint64_t retval;
    ret = xbee_at_local_cmd_read("%V", &retval, 0x01);
    if(ret != 2) {
        /* Return error */
        return ret;
    }
    /* Copy temperature reading */
    *vss = (uint16_t)retval;
    /* Seem like everything worked */
    return XBEE_RET_OK;
}


/*
 * Write the destination address (DH & DL)
 */
int8_t xbee_cmd_set_destination(uint32_t dh, uint32_t dl) {
    int8_t ret;
    uint64_t retval;
    
    /* Write the higher address word */
    ret = xbee_at_local_cmd_write("DH", dh, 0x01);
    if(ret < XBEE_RET_OK) {
        /* Command failed -> Error */
        return ret;
    }
    
    /* Write the lower address word */
    ret = xbee_at_local_cmd_write("DL", dl, 0x01);
    if(ret < XBEE_RET_OK) {
        /* Command failed -> Error */
        return ret;
    }
    
    /* Confirm writing */
    ret = xbee_at_local_cmd_read("WR", &retval, 0x01);
    if(ret < XBEE_RET_OK) {
        /* Command failed -> Error */
        return ret;
    }
    
    /* Seem like everything worked */
    return XBEE_RET_OK;
}


/*
 * Write the destination address to broadcast address
 */
int8_t xbee_cmd_set_broadcast(void) {
    /* Set the broadcast DH & DL addresses */
    return xbee_cmd_set_destination(XBEE_DH_BROADCAST,XBEE_DL_BROADCAST);
}


/**************************************/
/*** MISC FUNCTIONALITY ***************/
/**************************************/

/*
 * Flush the receive buffer
 */
void xbee_flush_rx(void) {
    /* Flush the UART RX buffer */
    uart_rx_flush();
}


/*
 * Flush the transmit buffer
 */
void xbee_flush_tx(void) {
    /* Flush the UART TX buffer */
    uart_tx_flush();
}
