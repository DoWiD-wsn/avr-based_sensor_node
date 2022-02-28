/*!
 * @brief   ASN(x) Xbee 3 library -- header file
 *
 * Library to the Xbee 3 module accessible via UART.
 *
 * @file    /_asnx_lib_/xbee/xbee.h
 * @author  Dominik Widhalm
 * @version 1.3.8
 * @date    2022/02/28
 */

#ifndef _ASNX_XBEE_H_
#define _ASNX_XBEE_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
#include <stddef.h>
/*** AVR ***/
#include <util/delay.h>
/*** ASNX LIB ***/
#include "hw/hw.h"


/***** DEFINES ********************************************************/
/* Xbee sleep-request pin (default) */
#define XBEE_SLEEP_REQ_DDR                  (DDRC)
#define XBEE_SLEEP_REQ_PORT                 (PORTC)
#define XBEE_SLEEP_REQ_PIN                  (PINC)
#define XBEE_SLEEP_REQ_GPIO                 (PC6)
/* Xbee sleep-indicator pin (default) */
#define XBEE_SLEEP_IND_DDR                  (DDRC)
#define XBEE_SLEEP_IND_PORT                 (PORTC)
#define XBEE_SLEEP_IND_PIN                  (PINC)
#define XBEE_SLEEP_IND_GPIO                 (PC7)

/*! Join network attempt retries */
#define XBEE_JOIN_RETRIES                   (200)
/*! Join network delay between tries [ms] */
#define XBEE_JOIN_DELAY                     (50)

/*! Wake-up attempt retries */
#define XBEE_WAKE_RETRIES                   (200)
/*! Wake-up delay between tries [ms] */
#define XBEE_WAKE_DELAY                     (25)

/*! Response receive attempts */
#define XBEE_RESPONSE_ATTEMPTS              (5)
/*! Response attempt retries */
#define XBEE_RESPONSE_RETRIES               (25)
/*! Response delay between tries [ms] */
#define XBEE_RESPONSE_DELAY                 (25)

/*! Maximum Number of Transmission Bytes (for transparent mode) */
#define XBEE_CONF_NP                        (54)
/*!
 * Unicast transmission maximum payload size.
 * XBee Zigbee firmware supports fragmentation that allows a single
 * large data packet to be broken up into multiple RF transmissions and
 * reassembled by the receiver before sending data out its serial port.
 */
#define XBEE_CONF_PAYLOAD_MAX               (84)

/* Network source 64-bit addressing */
#define XBEE_ADDR64_USE16                   (0x000000000000FFFF)
#define XBEE_ADDR64_BROADCAST               (0xFFFFFFFFFFFFFFFF)
#define XBEE_ADDR64_COORDINATOR             (0x0000000000000000)
/* Network source 16-bit addressing */
#define XBEE_ADDR16_UKNOWN                  (0xFFFE)
#define XBEE_ADDR16_ALL_ROUTER              (0xFFFC)
#define XBEE_ADDR16_ALL_NON_SLEEP           (0xFFFD)
#define XBEE_ADDR16_ALL_DEVICES             (0xFFFF)
#define XBEE_ADDR16_COORDINATOR             (0x0000)
#define XBEE_ADDR16_USE64                   (XBEE_ADDR16_UKNOWN)
/* DH - Destination Address High */
#define XBEE_DH_BROADCAST                   (0x00000000)
#define XBEE_DH_COORDINATOR                 (0x00000000)
/* DL - Destination Address Low */
#define XBEE_DL_BROADCAST                   (0x0000FFFF)
#define XBEE_DL_COORDINATOR                 (0x00000000)

/* Xbee API mode delimiter */
#define XBEE_START_DELIMITER                (0x7E)

/* Remote command options */
#define XBEE_ATCMD_REMOTE_OPT_NO            (0x00)
#define XBEE_ATCMD_REMOTE_OPT_NACK          (0x01)
#define XBEE_ATCMD_REMOTE_OPT_APPLY         (0x02)
#define XBEE_ATCMD_REMOTE_OPT_SECURE        (0x10)
#define XBEE_ATCMD_REMOTE_OPT               (XBEE_ATCMD_REMOTE_OPT_APPLY)

/* Broadcast transmit radius */
#define XBEE_TRANSMIT_BROADCAST_RADIUS      (0x00)
/* Transmit command options */
#define XBEE_TRANSMIT_OPT_NO                (0x00)
#define XBEE_TRANSMIT_OPT_NACK              (0x01)
#define XBEE_TRANSMIT_OPT_INDIRECT_BINDING  (0x04)
#define XBEE_TRANSMIT_OPT_MULTICAST         (0x08)
#define XBEE_TRANSMIT_OPT_SECURE            (0x10)
#define XBEE_TRANSMIT_OPT_APS_ENCRYPTION    (0x20)
#define XBEE_TRANSMIT_OPT_EXT_TIMEOUT       (0x40)
#define XBEE_TRANSMIT_OPT                   (XBEE_TRANSMIT_OPT_NO)


/***** ENUMERATION ****************************************************/
/*! Enumeration for the Xbee function returns values */
typedef enum {
    XBEE_RET_OK                         = 0,    /**< SUCCESS */
    XBEE_RET_ERROR                      = -1,   /**< ERROR: general error */
    XBEE_RET_INVALID_CMD                = -2,   /**< ERROR: invalid command given */
    XBEE_RET_INVALID_PARAM              = -3,   /**< ERROR: invalid parameter given */
    XBEE_RET_FAILED_TRANSMISSION        = -4,   /**< ERROR: transmission failed */
    XBEE_RET_NOT_SECURE                 = -5,   /**< ERROR: no secure connection */
    XBEE_RET_ENCRYPTION_ERROR           = -6,   /**< ERROR: encryption error */
    XBEE_RET_CMD_NOT_SECURE             = -7,   /**< ERROR: command not secure */
    XBEE_RET_INVALID_CRC                = -8,   /**< ERROR: CRC check failed */
    XBEE_RET_INVALID_FRAME              = -9,   /**< ERROR: invalid frame given */
    XBEE_RET_PAYLOAD_SIZE_EXCEEDED      = -10,  /**< ERROR: maximum payload size exceeded */
    XBEE_RET_FID_NOT_MATCH              = -11,  /**< ERROR: returned FID does not match */
    XBEE_RET_MAC_NOT_MATCH              = -12,  /**< ERROR: returned MAC does not match */
    XBEE_RET_ADDR_NOT_MATCH             = -13,  /**< ERROR: returned address does not match */
    XBEE_RET_TIMEOUT                    = -14   /**< ERROR: timeout */
} XBEE_RET_t;

/*! Enumeration for the Xbee frame and response types */
typedef enum {
    XBEE_FRAME_ATCMD_LOCAL              = 0x08, /**< Local AT Command Request (0x08) */
    XBEE_FRAME_ATCMD_LOCAL_QUEUE        = 0x09, /**< Queue Local AT Command Request (0x09) */
    XBEE_FRAME_TRANSMIT                 = 0x10, /**< Transmit Request (0x10) */
    XBEE_FRAME_EXP_ADDRESSING           = 0x11, /**< Explicit Addressing Command Request (0x11) */
    XBEE_FRAME_ATCMD_REMOTE             = 0x17, /**< Remote AT Command Request (0x17) */
    XBEE_FRAME_CREATE_SRC_ROUTE         = 0x21, /**< Create Source Route (0x21) */
    XBEE_FRAME_REGISTER_DEVICE          = 0x24, /**< Register Joining Device (0x24) */
    XBEE_FRAME_BLE_UNLOCK               = 0x2C, /**< BLE Unlock Request (0x2C) */
    XBEE_FRAME_USER_DATA_RELAY_IN       = 0x2D, /**< User Data Relay Input (0x2D) */
    XBEE_FRAME_SEC_SESSION_CTRL         = 0x2E, /**< Secure Session Control (0x2E) */
    /****/
    XBEE_FRAME_MODEM_STATUS             = 0x8A, /**< Modem Status (0x8A) */
    XBEE_FRAME_TRANSMIT_STATUS_EXT      = 0x8B, /**< Extended Transmit Status (0x8B) */
    XBEE_FRAME_ATCMD_LOCAL_RESP         = 0x88, /**< Local AT Command Response (0x88) */
    XBEE_FRAME_TRANSMIT_STATUS          = 0x89, /**< Transmit Status (0x89) */
    XBEE_FRAME_RECEIVE                  = 0x90, /**< Receive Packet (0x90) */
    XBEE_FRAME_RECEIVE_EXP_INDICATOR    = 0x91, /**< Explicit Receive Indicator (0x91) */
    XBEE_FRAME_IO_SAMPLE_INDICATOR      = 0x92, /**< I/O Sample Indicator (0x92) */
    XBEE_FRAME_NODE_IDENT_INDICATOR     = 0x95, /**< Node Identification Indicator (0x95) */
    XBEE_FRAME_ATCMD_REMOTE_RESP        = 0x97, /**< Remote AT Command Response (0x97) */
    XBEE_FRAME_MODEM_STATUS_EXT         = 0x98, /**< Extended Modem Status (0x98) */
    XBEE_FRAME_ROUTE_REC_INDICATOR      = 0xA1, /**< Route Record Indicator (0xA1) */
    XBEE_FRAME_MTO_ROUTE_INDICATOR      = 0xA3, /**< Many-to-One Route Request Indicator (0xA3) */
    XBEE_FRAME_BLE_UNLOCK_RESP          = 0xAC, /**< BLE Unlock Response (0xAC) */
    XBEE_FRAME_SEC_SESSION_RESP         = 0xAE, /**< Secure Session Response (0xAE) */
} XBEE_FRAME_t;
/*! Defines for double values */
#define XBEE_FRAME_REGISTER_STATUS      XBEE_FRAME_REGISTER_DEVICE

/*! Enumeration for the Xbee modem statuses */
typedef enum {
    XBEE_MODEM_STATUS_HW_RESET          = 0x00, /**< Hardware reset or power up (0x00) */
    XBEE_MODEM_STATUS_WD_RESTE          = 0x01, /**< Watchdog timer reset (0x01) */
    XBEE_MODEM_STATUS_JOINED            = 0x02, /**< Joined network (routers and end devices; 0x02) */
    XBEE_MODEM_STATUS_DISASSOCIATED     = 0x03, /**< Disassociated (0x03) */
    XBEE_MODEM_STATUS_CORD_STARTED      = 0x06, /**< Coordinator started (0x06) */
    XBEE_MODEM_STATUS_KEY_UPDATED       = 0x07, /**< Network security key was updated (0x07) */
    XBEE_MODEM_STATUS_CSS_LIMIT         = 0x0D, /**< Voltage supply limit exceeded (0x0D) */
    XBEE_MODEM_STATUS_CFG_CHANGED       = 0x11, /**< Modem configuration changed while join in progress (0x11) */
    XBEE_MODEM_STATUS_SEC_ESTABLISHED   = 0x3B, /**< Secure session successfully established (0x3B) */
    XBEE_MODEM_STATUS_SEC_ENDED         = 0x3C, /**< Secure session ended (0x3C) */
    XBEE_MODEM_STATUS_SEC_AUTH_FAILED   = 0x3D, /**< Secure session authentication failed (0x3D) */
    XBEE_MODEM_STATUS_PAN_CONFLICT      = 0x3E, /**< Coordinator detected a PAN ID conflict but because CR = 0 (0x3E) */
    XBEE_MODEM_STATUS_PAN_ID_CHANGED    = 0x3F, /**< Coordinator changed PAN ID due to a conflict (0x3F) */
    XBEE_MODEM_STATUS_BLE_CONNECT       = 0x32, /**< BLE Connect (0x32) */
    XBEE_MODEM_STATUS_BLE_DISCONNECT    = 0x33, /**< BLE Disconnect (0x33) */
    XBEE_MODEM_STATUS_NO_SEC_SESSION    = 0x34, /**< No Secure Session Connection (0x34) */
    XBEE_MODEM_STATUS_ROUTER_ID_CHANGED = 0x40, /**< Router PAN ID was changed by coordinator due to a conflict (0x40) */
    XBEE_MODEM_STATUS_NET_WD_EXPIRED    = 0x42, /**< Network Watchdog timeout expired three times (0x42) */
    /* 0x80-0xFF = Stack error */
} XBEE_MODEM_STATUS_t;

/*! Enumeration for the association indication response statuses */
typedef enum {
    XBEE_AI_RET_SUCCESS                 = 0x00, /**< Successfully formed or joined a Zigbee network (0x00) */
    XBEE_AI_RET_NO_PAN                  = 0x21, /**< Scan found no PANs (0x21) */
    XBEE_AI_RET_NO_VALID_PAN            = 0x22, /**< Scan found no valid PANs based on SC and ID settings (0x22) */
    XBEE_AI_RET_JOIN_DISABLED           = 0x23, /**< Valid PAN found, but joining is currently disabled (0x23) */
    XBEE_AI_RET_NO_BEACON               = 0x24, /**< No joinable beacons were found (0x24) */
    XBEE_AI_RET_JOIN_FAIL               = 0x27, /**< Join attempt failed (0x27) */
    XBEE_AI_RET_CE_START_FAIL           = 0x2A, /**< Failed to start coordinator (0x2A) */
    XBEE_AI_RET_CHECKING_FOR_CE         = 0x2B, /**< Checking for existing coordinator (0x2B) */
    XBEE_AI_RET_SEC_ATTACHED_WAIT_KEY   = 0x40, /**< Secure Join - Successfully attached to network, waiting for new link key (0x40) */
    XBEE_AI_RET_SEC_GOT_KEY             = 0x41, /**< Secure Join - Successfully received new link key from the trust center (0x41) */
    XBEE_AI_RET_SEC_KEY_EXCHANGE_FAIL   = 0x44, /**< Secure Join - Failed to receive new link key from the trust center (0x44) */
    XBEE_AI_RET_NO_RESPONSE             = 0xAB, /**< Attempted to join a device that did not respond (0xAB) */
    XBEE_AI_RET_SEC_NO_KEY              = 0xAD, /**< Secure Join - a network security key was not received from the trust center (0xAD) */
    XBEE_AI_RET_SEC_NEED_KEY            = 0xAF, /**< Secure Join - a preconfigured key is required to join the network (0xAF) */
    XBEE_AI_RET_INIT                    = 0xFF, /**< Initialization time; no association status has been determined yet (0xFF) */
} XBEE_AI_RET_t;

/*! Enumeration for the extended transmit status codes */
typedef enum {
    XBEE_TRANSMIT_STAT_DEL_OK           = 0x00, /**< Success () */
    XBEE_TRANSMIT_STAT_DEL_MAC_ACK_FAIL = 0x01, /**< MAC ACK failure (0x01) */
    XBEE_TRANSMIT_STAT_DEL_CCA_LBT_FAIL = 0x02, /**< CCA/LBT failure (0x02) */
    XBEE_TRANSMIT_STAT_DEL_MSG_UNREQ_SP = 0x03, /**< Indirect message unrequested / no spectrum available (0x03) */
    XBEE_TRANSMIT_STAT_DEL_INV_ENDPOINT = 0x15, /**< Invalid destination endpoint (0x15) */
    XBEE_TRANSMIT_STAT_DEL_NET_ACK_FAIL = 0x21, /**< Network ACK failure (0x21) */
    XBEE_TRANSMIT_STAT_DEL_NOT_JOINED   = 0x22, /**< Not joined to network (0x22) */
    XBEE_TRANSMIT_STAT_DEL_SELF_ADDRESS = 0x23, /**< Self-addressed (0x23) */
    XBEE_TRANSMIT_STAT_DEL_NO_ADDR      = 0x24, /**< Address not found (0x24) */
    XBEE_TRANSMIT_STAT_DEL_NO_ROUTE     = 0x25, /**< Route not found (0x25) */
    XBEE_TRANSMIT_STAT_DEL_BC_SRC_FAIL  = 0x26, /**< Broadcast source failed to hear a neighbor relay the message (0x26) */
    XBEE_TRANSMIT_STAT_DEL_INV_BINDING  = 0x2B, /**< Invalid binding table index (0x2B) */
    XBEE_TRANSMIT_STAT_DEL_RES_ERR      = 0x2C, /**< Resource error - lack of free buffers, timers, etc. (0x2C) */
    XBEE_TRANSMIT_STAT_DEL_BR_APS_TRANS = 0x2D, /**< Attempted broadcast with APS transmission (0x2D) */
    XBEE_TRANSMIT_STAT_DEL_UN_APS_TRANS = 0x2E, /**< Attempted unicast with APS transmission, but EE = 0 (0x2E) */
    XBEE_TRANSMIT_STAT_DEL_INT_RES_ERR  = 0x31, /**< Internal resource error (0x31) */
    XBEE_TRANSMIT_STAT_DEL_RES_LACK_ERR = 0x32, /**< Resource error lack of free buffers, timers, etc. (0x32) */
    XBEE_TRANSMIT_STAT_DEL_NOT_SECURE   = 0x34, /**< No Secure Session connection (0x34) */
    XBEE_TRANSMIT_STAT_DEL_ENC_FAIL     = 0x35, /**< Encryption failure (0x34) */
    XBEE_TRANSMIT_STAT_DEL_PAYLOAD_SIZE = 0x74, /**< Data payload too large (0x74) */
    XBEE_TRANSMIT_STAT_DEL_MSG_UNREQ    = 0x75, /**< Indirect message unrequested (0x75) */
    /*** DISCOVERY ***/
    XBEE_TRANSMIT_STAT_DIS_NO_OVERHEAD  = 0x00, /**< No discovery overhead (0x00) */
    XBEE_TRANSMIT_STAT_DIS_ZIGBEE       = 0x01, /**< Zigbee address discovery (0x01) */
    XBEE_TRANSMIT_STAT_DIS_ROUTE        = 0x02, /**< Route discovery (0x02) */
    XBEE_TRANSMIT_STAT_DIS_ZIGBEE_ROUTE = 0x03, /**< Zigbee address and route discovery (0x03) */
    XBEE_TRANSMIT_STAT_DIS_TIMEOUT      = 0x40, /**< Zigbee end device extended timeout (0x40) */
} XBEE_TRANSMIT_STAT_t;


/***** FUNCTION PROTOTYPES ****************************************************/
void xbee_init(void (*write)(uint8_t byte), int8_t (*read)(uint8_t* byte), uint8_t (*available)(void));
void xbee_set_sleep_request_gpio(volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin);
void xbee_set_sleep_indicator_gpio(volatile uint8_t* ddr, volatile uint8_t* port, volatile uint8_t* pin, uint8_t portpin);
XBEE_RET_t xbee_sleep_enable(void);
XBEE_RET_t xbee_sleep_disable(void);

/*** FRAMES ***/
/* Frame ID */
uint8_t fid_get_next(void);
void fid_reset(void);
/* Local AT commands */
XBEE_RET_t xbee_at_local_cmd_write(char* command, uint64_t value);
XBEE_RET_t xbee_at_local_cmd_read(char* command, uint64_t* value);
/* Remote AT commands */
XBEE_RET_t xbee_at_remote_cmd_write(uint64_t mac, uint16_t addr, char* command, uint64_t value);
XBEE_RET_t xbee_at_remote_cmd_read(uint64_t mac, uint16_t addr, char* command, uint64_t* value);
/* Transmit */
XBEE_RET_t xbee_transmit(uint64_t mac, uint16_t addr, uint8_t* payload, uint16_t cnt, uint8_t fid);
XBEE_RET_t xbee_transmit_status(uint8_t* delivery);
XBEE_RET_t xbee_transmit_status_ext(uint16_t* addr, uint8_t* retries, uint8_t* delivery, uint8_t* discovery, uint8_t* fid);
/* CRC checks */
uint8_t xbee_get_crc(uint8_t* data, uint8_t len);
XBEE_RET_t xbee_check_crc(uint8_t* data, uint8_t len, uint8_t crc);

/*** COMMON ***/
XBEE_RET_t xbee_transmit_broadcast(uint8_t* payload, uint16_t cnt, uint8_t fid);
XBEE_RET_t xbee_transmit_unicast(uint64_t mac, uint8_t* payload, uint16_t cnt, uint8_t fid);
XBEE_RET_t xbee_is_connected(void);
XBEE_RET_t xbee_wait_for_connected(void);
XBEE_RET_t xbee_cmd_get_temperature(float* temp);
XBEE_RET_t xbee_cmd_get_vss(float* vss);

#endif // _ASNX_XBEE_H_
