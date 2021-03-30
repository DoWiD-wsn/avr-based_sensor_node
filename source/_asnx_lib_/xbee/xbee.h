/**
 *  Header file for Xbee 3 functionality.
 */

#ifndef _AVR_XBEE_H_
#define _AVR_XBEE_H_

/***** INCLUDES ***************************************************************/
#include <stdint.h>


/***** MACROS *****************************************************************/
/*** TIMEOUT ***/
/* RX timeout [ms] */
#define XBEE_RX_TIMEOUT                     (1000)
#define XBEE_RX_TIMEOUT_DELAY               (10)

/*** CONFIGURATION ***/
// Use blocking (0) or non-blocking (1) UART write
#define XBEE_WRITE_NONBLOCKING              (0)
// Maximum Number of Transmission Bytes (for transparent mode)
#define XBEE_CONF_NP                        (54)
// Unicast transmission maximum payload size
/* XBee Zigbee firmware supports fragmentation that allows a single large data packet to be broken up into multiple RF transmissions and reassembled by the receiver before sending data out its serial port. */
#define XBEE_CONF_PAYLOAD_MAX               (84)

/*** ADDRESSING ***/
/* 64-bit */
#define XBEE_ADDR64_USE16                   (0x000000000000FFFF)
#define XBEE_ADDR64_BROADCAST               (0xFFFFFFFFFFFFFFFF)
#define XBEE_ADDR64_COORDINATOR             (0x0000000000000000)
/* 16-bit */
#define XBEE_ADDR16_USE64                   (XBEE_ADDR16_UKNOWN)
#define XBEE_ADDR16_UKNOWN                  (0xFFFE)
#define XBEE_ADDR16_ALL_ROUTER              (0xFFFC)
#define XBEE_ADDR16_ALL_NON_SLEEP           (0xFFFD)
#define XBEE_ADDR16_ALL_DEVICES             (0xFFFF)
#define XBEE_ADDR16_COORDINATOR             (0x0000)
/* DH - Destination Address High */
#define XBEE_DH_BROADCAST                   (0x00000000)
#define XBEE_DH_COORDINATOR                 (0x00000000)
/* DL - Destination Address Low */
#define XBEE_DL_BROADCAST                   (0x0000FFFF)
#define XBEE_DL_COORDINATOR                 (0x00000000)

/*** XBEE API ***/
#define XBEE_START_DELIMITER                (0x7E)

/*** XBEE FRAME TYPES & RESPONSES ***/
// Local AT Command Request - 0x08
#define XBEE_FRAME_ATCMD_LOCAL              (0x08)
// Queue Local AT Command Request - 0x09
#define XBEE_FRAME_ATCMD_LOCAL_QUEUE        (0x09)
// Transmit Request - 0x10
#define XBEE_FRAME_TRANSMIT                 (0x10)
// Explicit Addressing Command Request - 0x11
#define XBEE_FRAME_EXP_ADDRESSING           (0x11)
// Remote AT Command Request - 0x17
#define XBEE_FRAME_ATCMD_REMOTE             (0x17)
// Create Source Route - 0x21
#define XBEE_FRAME_CREATE_SRC_ROUTE         (0x21)
// Register Joining Device - 0x24
#define XBEE_FRAME_REGISTER_DEVICE          (0x24)
// Registration Status - 0x24
#define XBEE_FRAME_REGISTER_STATUS          (0x24)
// BLE Unlock Request - 0x2C
#define XBEE_FRAME_BLE_UNLOCK               (0x2C)
// User Data Relay Input - 0x2D
#define XBEE_FRAME_USER_DATA_RELAY_IN       (0x2D)
// Secure Session Control - 0x2E
#define XBEE_FRAME_SEC_SESSION_CTRL         (0x2E)
/****/
// Modem Status - 0x8A
#define XBEE_FRAME_MODEM_STATUS             (0x8A)
// Extended Transmit Status - 0x8B
#define XBEE_FRAME_TRANSMIT_STATUS_EXT      (0x8B)
// Local AT Command Response - 0x88
#define XBEE_FRAME_ATCMD_LOCAL_RESP         (0x88)
// Transmit Status - 0x89
#define XBEE_FRAME_TRANSMIT_STATUS          (0x89)
// Receive Packet - 0x90
#define XBEE_FRAME_RECEIVE                  (0x90)
// Explicit Receive Indicator - 0x91
#define XBEE_FRAME_RECEIVE_EXP_INDICATOR    (0x91)
// I/O Sample Indicator - 0x92
#define XBEE_FRAME_IO_SAMPLE_INDICATOR      (0x92)
// Node Identification Indicator - 0x95
#define XBEE_FRAME_NODE_IDENT_INDICATOR     (0x95)
// Remote AT Command Response- 0x97
#define XBEE_FRAME_ATCMD_REMOTE_RESP        (0x97)
// Extended Modem Status - 0x98
#define XBEE_FRAME_MODEM_STATUS_EXT         (0x98)
// Route Record Indicator - 0xA1
#define XBEE_FRAME_ROUTE_REC_INDICATOR      (0xA1)
// Many-to-One Route Request Indicator - 0xA3
#define XBEE_FRAME_MTO_ROUTE_INDICATOR      (0xA3)
// BLE Unlock Response - 0xAC
#define XBEE_FRAME_BLE_UNLOCK_RESP          (0xAC)
// Secure Session Response - 0xAE
#define XBEE_FRAME_SEC_SESSION_RESP         (0xAE)

/*** XBEE MODEM STATUS ***/
// Hardware reset or power up
#define XBEE_MODEM_STATUS_HW_RESET          (0x00)
// Watchdog timer reset
#define XBEE_MODEM_STATUS_WD_RESTE          (0x01)
// Joined network (routers and end devices)
#define XBEE_MODEM_STATUS_JOINED            (0x02)
// Disassociated
#define XBEE_MODEM_STATUS_DISASSOCIATED     (0x03)
// Coordinator started
#define XBEE_MODEM_STATUS_CORD_STARTED      (0x06)
// Network security key was updated
#define XBEE_MODEM_STATUS_KEY_UPDATED       (0x07)
// Voltage supply limit exceeded
#define XBEE_MODEM_STATUS_CSS_LIMIT         (0x0D)
// Modem configuration changed while join in progress
#define XBEE_MODEM_STATUS_CFG_CHANGED       (0x11)
// XBee 3 - Secure session successfully established
#define XBEE_MODEM_STATUS_SEC_ESTABLISHED   (0x3B)
// XBee 3 - Secure session ended
#define XBEE_MODEM_STATUS_SEC_ENDED         (0x3C)
// XBee 3 - Secure session authentication failed
#define XBEE_MODEM_STATUS_SEC_AUTH_FAILED   (0x3D)
// XBee 3 - Coordinator detected a PAN ID conflict but because CR = 0, no action will be taken.
#define XBEE_MODEM_STATUS_PAN_CONFLICT      (0x3E)
// Coordinator changed PAN ID due to a conflict
#define XBEE_MODEM_STATUS_PAN_ID_CHANGED    (0x3F)
// XBee 3 - BLE Connect
#define XBEE_MODEM_STATUS_BLE_CONNECT       (0x32)
// XBee 3 - BLE Disconnect
#define XBEE_MODEM_STATUS_BLE_DISCONNECT    (0x33)
// XBee 3 - No Secure Session Connection
#define XBEE_MODEM_STATUS_NO_SEC_SESSION    (0x34)
// Router PAN ID was changed by coordinator due to a conflict
#define XBEE_MODEM_STATUS_ROUTER_ID_CHANGED (0x40)
// Network Watchdog timeout expired three times
#define XBEE_MODEM_STATUS_NET_WD_EXPIRED    (0x42)
// 0x80 - 0xFF = Stack error

/*** Association Indication Response ***/
// Successfully formed or joined a Zigbee network.
#define XBEE_AI_RET_SUCCESS                 (0x00)
// Scan found no PANs.
#define XBEE_AI_RET_NO_PAN                  (0x21)
// Scan found no valid PANs based on SC and ID settings.
#define XBEE_AI_RET_NO_VALID_PAN            (0x22)
// Valid PAN found, but joining is currently disabled.
#define XBEE_AI_RET_JOIN_DISABLED           (0x23)
// No joinable beacons were found.
#define XBEE_AI_RET_NO_BEACON               (0x24)
// Join attempt failed.
#define XBEE_AI_RET_JOIN_FAIL               (0x27)
// Failed to start coordinator.
#define XBEE_AI_RET_CE_START_FAIL           (0x2A)
// Checking for existing coordinator.
#define XBEE_AI_RET_CHECKING_FOR_CE         (0x2B)
// Secure Join - Successfully attached to network, waiting for new link key.
#define XBEE_AI_RET_SEC_ATTACHED_WAIT_KEY   (0x40)
// Secure Join - Successfully received new link key from the trust center.
#define XBEE_AI_RET_SEC_GOT_KEY             (0x41)
// Secure Join - Failed to receive new link key from the trust center.
#define XBEE_AI_RET_SEC_KEY_EXCHANGE_FAIL   (0x44)
// Attempted to join a device that did not respond.
#define XBEE_AI_RET_NO_RESPONSE             (0xAB)
// Secure Join - a network security key was not received from the trust center.
#define XBEE_AI_RET_SEC_NO_KEY              (0xAD)
// Secure Join - a preconfigured key is required to join the network.
#define XBEE_AI_RET_SEC_NEED_KEY            (0xAF)
// Initialization time; no association status has been determined yet.
#define XBEE_AI_RET_INIT                    (0xFF)

/*** REMOTE COMMAND OPTIONS ***/
#define XBEE_ATCMD_REMOTE_OPT               (XBEE_ATCMD_REMOTE_OPT_APPLY)
#define XBEE_ATCMD_REMOTE_OPT_NO            (0x00)
#define XBEE_ATCMD_REMOTE_OPT_NACK          (0x01)
#define XBEE_ATCMD_REMOTE_OPT_APPLY         (0x02)
#define XBEE_ATCMD_REMOTE_OPT_SECURE        (0x10)

/*** TRANSMIT COMMAND OPTIONS ***/
#define XBEE_TRANSMIT_BROADCAST_RADIUS      (0x00)
#define XBEE_TRANSMIT_OPT                   (XBEE_TRANSMIT_OPT_NO)
#define XBEE_TRANSMIT_OPT_NO                (0x00)
#define XBEE_TRANSMIT_OPT_NACK              (0x01)
#define XBEE_TRANSMIT_OPT_INDIRECT_BINDING  (0x04)
#define XBEE_TRANSMIT_OPT_MULTICAST         (0x08)
#define XBEE_TRANSMIT_OPT_SECURE            (0x10)
#define XBEE_TRANSMIT_OPT_APS_ENCRYPTION    (0x20)
#define XBEE_TRANSMIT_OPT_EXT_TIMEOUT       (0x40)

/*** EXTENDED TRANSMIT STATUS CODES ***/
/*** DELIVERY ***/
// Success
#define XBEE_TRANSMIT_STAT_DEL_OK           (0x00)
// MAC ACK failure
#define XBEE_TRANSMIT_STAT_DEL_MAC_ACK_FAIL (0x01)
// CCA/LBT failure
#define XBEE_TRANSMIT_STAT_DEL_CCA_LBT_FAIL (0x02)
// Indirect message unrequested / no spectrum available
#define XBEE_TRANSMIT_STAT_DEL_MSG_UNREQ_SP (0x03)
// Invalid destination endpoint
#define XBEE_TRANSMIT_STAT_DEL_INV_ENDPOINT (0x15)
// Network ACK failure
#define XBEE_TRANSMIT_STAT_DEL_NET_ACK_FAIL (0x21)
// Not joined to network
#define XBEE_TRANSMIT_STAT_DEL_NOT_JOINED   (0x22)
// Self-addressed
#define XBEE_TRANSMIT_STAT_DEL_SELF_ADDRESS (0x23)
// Address not found
#define XBEE_TRANSMIT_STAT_DEL_NO_ADDR      (0x24)
// Route not found
#define XBEE_TRANSMIT_STAT_DEL_NO_ROUTE     (0x25)
// Broadcast source failed to hear a neighbor relay the message
#define XBEE_TRANSMIT_STAT_DEL_BC_SRC_FAIL  (0x26)
// Invalid binding table index
#define XBEE_TRANSMIT_STAT_DEL_INV_BINDING  (0x2B)
// Resource error - lack of free buffers, timers, etc.
#define XBEE_TRANSMIT_STAT_DEL_RES_ERR      (0x2C)
// Attempted broadcast with APS transmission
#define XBEE_TRANSMIT_STAT_DEL_BR_APS_TRANS (0x2D)
// Attempted unicast with APS transmission, but EE = 0
#define XBEE_TRANSMIT_STAT_DEL_UN_APS_TRANS (0x2E)
// Internal resource error
#define XBEE_TRANSMIT_STAT_DEL_INT_RES_ERR  (0x31)
// Resource error lack of free buffers, timers, etc.
#define XBEE_TRANSMIT_STAT_DEL_RES_LACK_ERR (0x32)
// No Secure Session connection
#define XBEE_TRANSMIT_STAT_DEL_NOT_SECURE   (0x34)
// Encryption failure
#define XBEE_TRANSMIT_STAT_DEL_ENC_FAIL     (0x35)
// Data payload too large
#define XBEE_TRANSMIT_STAT_DEL_PAYLOAD_SIZE (0x74)
// Indirect message unrequested
#define XBEE_TRANSMIT_STAT_DEL_MSG_UNREQ    (0x75)
/*** DISCOVERY ***/
// No discovery overhead
#define XBEE_TRANSMIT_STAT_DIS_NO_OVERHEAD  (0x00)
// Zigbee address discovery
#define XBEE_TRANSMIT_STAT_DIS_ZIGBEE       (0x01)
// Route discovery
#define XBEE_TRANSMIT_STAT_DIS_ROUTE        (0x02)
// Zigbee address and route discovery
#define XBEE_TRANSMIT_STAT_DIS_ZIGBEE_ROUTE (0x03)
// Zigbee end device extended timeout
#define XBEE_TRANSMIT_STAT_DIS_TIMEOUT      (0x40)

/*** FUNCTION RETURN VALUES ***/
#define XBEE_RET_OK                         (0)
#define XBEE_RET_ERROR                      (-1)
#define XBEE_RET_INVALID_CMD                (-2)
#define XBEE_RET_INVALID_PARAM              (-3)
#define XBEE_RET_FAILED_TRANSMISSION        (-4)
#define XBEE_RET_NOT_SECURE                 (-5)
#define XBEE_RET_ENCRYPTION_ERROR           (-6)
#define XBEE_RET_CMD_NOT_SECURE             (-7)
#define XBEE_RET_INVALID_CRC                (-8)
#define XBEE_RET_INVALID_FRAME              (-9)
#define XBEE_RET_PAYLOAD_SIZE_EXCEEDED      (-10)
#define XBEE_RET_FID_NOT_MATCH              (-11)
#define XBEE_RET_MAC_NOT_MATCH              (-12)
#define XBEE_RET_ADDR_NOT_MATCH             (-13)


/***** GLOBAL VARIABLES *******************************************************/


/***** ENUMERATION ************************************************************/


/***** STRUCTURES *************************************************************/


/***** FUNCTION PROTOTYPES ****************************************************/
void xbee_init(uint32_t baud);
/***** FRAMES *************************/
/* Local AT Commands */
int8_t xbee_at_local_cmd_write(char* command, uint64_t value, uint8_t fid);
int8_t xbee_at_local_cmd_read(char* command, uint64_t* value, uint8_t fid);
/* Remote AT Commands */
int8_t xbee_at_remote_cmd_write(uint64_t mac, uint16_t addr, char* command, uint64_t value, uint8_t fid);
int8_t xbee_at_remote_cmd_read(uint64_t mac, uint16_t addr, char* command, uint64_t* value, uint8_t fid);
/* Transmit */
int8_t xbee_transmit(uint64_t mac, uint16_t addr, uint8_t* payload, uint16_t cnt, uint8_t fid);
int8_t xbee_transmit_status(uint8_t* delivery);
int8_t xbee_transmit_status_ext(uint16_t* addr, uint8_t* retries, uint8_t* delivery, uint8_t* discovery, uint8_t* fid);

/***** FRAME-related ******************/
uint8_t xbee_get_crc(uint8_t* data, uint8_t len);
uint8_t xbee_check_crc(uint8_t* data, uint8_t len, uint8_t crc);

/***** COMMON FUNCTIONALITY ***********/
int8_t xbee_cmd_get_temperature(int16_t* temp);
int8_t xbee_cmd_get_vss(uint16_t* vss);
int8_t xbee_cmd_set_destination(uint32_t dh, uint32_t dl);
int8_t xbee_cmd_set_broadcast(void);

/***** MISC *****/
void xbee_flush_rx(void);
void xbee_flush_tx(void);


/***** INLINE FUNCTIONS *******************************************************/


#endif // _AVR_XBEE_H_
