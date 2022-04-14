/*!
 * @brief   ASN(x) ZMOD4410 gas sensor library -- header file
 *
 * Library to support the ZMOD4410 gas sensor.
 *
 * @file    /_asnx_lib_/sensors/zmod4410.h
 * @author  Dominik Widhalm
 * @version 1.0.0
 * @date    2022/04/14
 *
 * @see     https://github.com/renesas/fsp/tree/master/ra/fsp
 */

#ifndef _ASNX_ZMOD4410_H_
#define _ASNX_ZMOD4410_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
/*** AVR ***/
#include <util/delay.h>
/*** ASNX LIB ***/
#include "i2c/i2c.h"


/***** DEFINES ********************************************************/
/*** I2C general ***/
/*! I2C default address */
#define ZMOD4410_I2C_ADDRESS            0x32

/*** Sensor specific ***/
/*! Maximum number of retries for polling */
#define ZMOD4410_POLLING_MAX            100

/*** I2C register addresses ***/
/*! PID register address */
#define ZMOD4410_ADDR_PID               0x00
/*! Configuration register address */
#define ZMOD4410_ADDR_CONF              0x20
/*! Production data register address */
#define ZMOD4410_ADDR_DATA              0x26
/*! Command register address */
#define ZMOD4410_ADDR_CMD               0x93
/*! Status register address */
#define ZMOD4410_ADDR_STATUS            0x94
/*! Tracking register address */
#define ZMOD4410_ADDR_TRACKING          0x3A
/*! Error register address */
#define ZMOD4410_ADDR_ERROR             0xB7

/*** I2C data register addresses ***/
/*! H measurement register address */
#define ZMOD4410_DATA_H_ADDR            0x40
/*! D measurement register address */
#define ZMOD4410_DATA_D_ADDR            0x50
/*! M measurement register address */
#define ZMOD4410_DATA_M_ADDR            0x60
/*! S measurement register address */
#define ZMOD4410_DATA_S_ADDR            0x68
/*! R measurement register address */
#define ZMOD4410_DATA_R_ADDR            0x97

/*** I2C data measurement commands ***/
/*! Measurement start command */
#define ZMOD4410_DATA_START             0x80
/*! Measurement stop command */
#define ZMOD4410_DATA_STOP              0x00

/*** Status register data masks ***/
/*! Sequencer running mask */
#define ZMOD4410_STATUS_SEQ_RUNNING     0x80
/*! Sleep timer enabled mask */
#define ZMOD4410_STATUS_SLP_ENABLED     0x40
/*! Alarm mask */
#define ZMOD4410_STATUS_ALARM           0x20
/*! Last executed sequencer step mask */
#define ZMOD4410_STATUS_SEQ_LAST        0x1F
/*! POR_event mask */
#define ZMOD4410_STATUS_POR_EVENT       0x80
/*! Access conflict mask */
#define ZMOD4410_STATUS_ACCESS_CONFLICT 0x40

/*** ZMOD4410 function return values ***/
#define ZMOD4410_RET_ERROR_CALCULATION  (-5)
#define ZMOD4410_RET_ERROR_TIMEOUT      (-4)
#define ZMOD4410_RET_ERROR_STATUS       (-3)
#define ZMOD4410_RET_ERROR_NODEV        (-2)
#define ZMOD4410_RET_ERROR              (-1)
#define ZMOD4410_RET_OK                 (0)


/***** STRUCTURES *****************************************************/
/**
 * @brief A single data set for the configuration
 */
typedef struct {
    uint8_t addr;           /**< Data set start address */
    uint8_t len;            /**< Data set length (bytes) */
    uint8_t *data_buf;      /**< Data set buffer start */
} ZMOD4410_CONF_DATA_t;

/**
 * @brief Structure to hold the gas sensor module configuration.
 */
typedef struct {
    ZMOD4410_CONF_DATA_t h; /**< Sensor configuration H data set */
    ZMOD4410_CONF_DATA_t d; /**< Sensor configuration D data set */
    ZMOD4410_CONF_DATA_t m; /**< Sensor configuration M data set */
    ZMOD4410_CONF_DATA_t s; /**< Sensor configuration S data set */
    ZMOD4410_CONF_DATA_t r; /**< Sensor configuration R data set */
} ZMOD4410_CONF_t;

/*!
 * A structure to store the ZMOD4410 sensor configuration.
 */
typedef struct {
    uint8_t address;        /**< I2C address */
    uint8_t config[6];      /**< Configuration parameter set */
    uint16_t mox_er;        /**< Sensor specific parameter */
    uint16_t mox_lr;        /**< Sensor specific parameter */
    uint8_t prod_data[7];   /**< Sensor production data */
    ZMOD4410_CONF_t* init;  /**< Pointer to the init configuration */
    ZMOD4410_CONF_t* meas;  /**< Pointer to the measurement configuration */
} ZMOD4410_t;

/*!
 * A structure to store the ZMOD4410 sensor measurements.
 */
typedef struct {
    float rmox[13];         /**< MOx resistance  */
    float log_rcda;         /**< log10 of CDA resistance for IAQ 2nd Gen  */
    float iaq;              /**< IAQ index */
    float tvoc;             /**< TVOC concentration (mg/m^3) */
    float etoh;             /**< EtOH concentration (ppm) */
    float eco2;             /**< eCO2 concentration (ppm) */
} ZMOD4410_DATA_t;


/***** FUNCTION PROTOTYPES ********************************************/
/* General */
int8_t zmod4410_init(ZMOD4410_t* dev, uint8_t address);
int8_t zmod4410_get_status(ZMOD4410_t* dev, uint8_t* status);
int8_t zmod4410_get_pid(ZMOD4410_t* dev, uint16_t* pid);
int8_t zmod4410_get_config(ZMOD4410_t* dev, uint8_t* conf);
int8_t zmod4410_get_production_data(ZMOD4410_t* dev, uint8_t* data);
int8_t zmod4410_get_error(ZMOD4410_t* dev, uint8_t* error);
/* Measurement */
int8_t zmod4410_start(ZMOD4410_t* dev);
int8_t zmod4410_stop(ZMOD4410_t* dev);
int8_t zmod4410_get_measurement(ZMOD4410_t* dev, ZMOD4410_DATA_t* data);

#endif // _ASNX_ZMOD4410_H_
