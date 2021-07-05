/*!
 * @brief   ASN(x) BME280 environmental sensor library -- header file
 *
 * Library to support the BME280 environmental sensor.
 *
 * @file    /_asnx_lib_/sensors/bme280.h
 * @author  Dominik Widhalm
 * @version 1.2.0
 * @date    2021/06/07
 *
 * @see     https://github.com/bitbank2/bme280
 * @see     https://github.com/BoschSensortec/BME280_driver
 */

#ifndef _ASNX_BME280_H_
#define _ASNX_BME280_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
/*** AVR ***/
#include <util/delay.h>
/*** ASNX LIB ***/
#include "i2c/i2c.h"


/***** DEFINES ********************************************************/
/*! I2C default address */
#define BME280_I2C_ADDRESS              0x76
/*! Timeour for sensor readings [ms] */
#define BME280_READ_TIMEOUT             500
/*! Configuration register reset value */
#define BME280_RESET_VALUE              0xB6
/* Status codes */
#define BME280_STATUS_MEASURING         0x08
#define BME280_STATUS_UPDATING          0x01


/***** ENUMERATION ****************************************************/
/*! Enumeration for the BME280 function return values */
typedef enum {
    BME280_RET_ERROR_NODEV  = -2,
    BME280_RET_ERROR        = -1,
    BME280_RET_OK           = 0
} BME280_RET_t;

/*! Enumeration for the I2C register addresses */
typedef enum {
    BME280_REG_CHIPID       = 0xD0,
    BME280_REG_RESET        = 0xE0,
    BME280_REG_CTRL_HUM     = 0xF2,
    BME280_REG_STATUS       = 0xF3,
    BME280_REG_CTRL_MEAS    = 0xF4,
    BME280_REG_CONFIG       = 0xF5,
    BME280_REG_P_MSB        = 0xF7,
    BME280_REG_P_LSB        = 0xF8,
    BME280_REG_P_XLSB       = 0xF9,
    BME280_REG_T_MSB        = 0xFA,
    BME280_REG_T_LSB        = 0xFB,
    BME280_REG_T_XLSB       = 0xFC,
    BME280_REG_H_MSB        = 0xFD,
    BME280_REG_H_LSB        = 0xFE,
    /* Temperature compensation value addresses */
    BME280_REG_DIG_T1       = 0x88,
    BME280_REG_DIG_T2       = 0x8A,
    BME280_REG_DIG_T3       = 0x8C,
    /* Pressure compensation value addresses */
    BME280_REG_DIG_P1       = 0x8E,
    BME280_REG_DIG_P2       = 0x90,
    BME280_REG_DIG_P3       = 0x92,
    BME280_REG_DIG_P4       = 0x94,
    BME280_REG_DIG_P5       = 0x96,
    BME280_REG_DIG_P6       = 0x98,
    BME280_REG_DIG_P7       = 0x9A,
    BME280_REG_DIG_P8       = 0x9C,
    BME280_REG_DIG_P9       = 0x9E,
    /* Humidty compensation value addresses */
    BME280_REG_DIG_H1       = 0xA1,
    BME280_REG_DIG_H2       = 0xE1,
    BME280_REG_DIG_H3       = 0xE3,
    BME280_REG_DIG_H4       = 0xE4,
    BME280_REG_DIG_H5       = 0xE5,
    BME280_REG_DIG_H6       = 0xE6,
    BME280_REG_DIG_H7       = 0xE7
} BME280_REG_t;

/*! Enumeration for the BME280 sampling modes */
typedef enum {
    BME280_SAMPLE_NONE      = 0,
    BME280_SAMPLE_1         = 1,
    BME280_SAMPLE_2         = 2,
    BME280_SAMPLE_4         = 3,
    BME280_SAMPLE_8         = 4,
    BME280_SAMPLE_16        = 5
} BME280_SAMPLE_t;

/*! Enumeration for the BME280 operation modes */
typedef enum {
    BME280_MODE_SLEEP       = 0,
    BME280_MODE_FORCED      = 1,
    BME280_MODE_NORMAL      = 2
} BME280_MODE_t;

/*! Enumeration for the BME280 standby settings */
typedef enum {
    BME280_STANDBY_0_5      = 0,
    BME280_STANDBY_62_5     = 1,
    BME280_STANDBY_125      = 2,
    BME280_STANDBY_250      = 3,
    BME280_STANDBY_500      = 4,
    BME280_STANDBY_1000     = 5,
    BME280_STANDBY_10       = 6,
    BME280_STANDBY_20       = 7
} BME280_STANDBY_t;

/*! Enumeration for the BME280 filter settings */
typedef enum {
    BME280_FILTER_OFF       = 0,
    BME280_FILTER_2         = 1,
    BME280_FILTER_4         = 2,
    BME280_FILTER_8         = 3,
    BME280_FILTER_16        = 4
} BME280_FILTER_t;

/*! Enumeration for the BME280 SPI settings */
typedef enum {
    BME280_SPI_OFF          = 0,
    BME280_SPI_ON           = 1
} BME280_SPI_t;


/***** STRUCTURES *****************************************************/
/*!
 * A structure to store the BME280 calibration values.
 */
typedef struct {
    /* temperature */
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    /* pressure */
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    /* humidity */
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} BME280_CALIB_t;

/*!
 * A structure to store the BME280 sensor configuration.
 */
typedef struct {
    uint8_t address;        /**< I2C address */
    int32_t t_fine;         /**< temperature fine reading */
    BME280_CALIB_t calib;   /**< sensor calibration values */
} BME280_t;


/***** FUNCTION PROTOTYPES ********************************************/
/* General */
BME280_RET_t bme280_init(BME280_t* bme, uint8_t address);
BME280_RET_t bme280_reset(BME280_t* bme);
BME280_RET_t bme280_spi_enable(BME280_t* bme);
BME280_RET_t bme280_spi_disable(BME280_t* bme);
/* Configuration - read */
BME280_RET_t bme280_get_chipid(BME280_t* bme, uint8_t* value);
BME280_RET_t bme280_get_ctrl_hum(BME280_t* bme, uint8_t* value);
BME280_RET_t bme280_get_status(BME280_t* bme, uint8_t* value);
BME280_RET_t bme280_get_ctrl_meas(BME280_t* bme, uint8_t* value);
BME280_RET_t bme280_get_config(BME280_t* bme, uint8_t* value);
/* Configuration  - write */
BME280_RET_t bme280_set_mode(BME280_t* bme, BME280_MODE_t value);
BME280_RET_t bme280_set_t_sample(BME280_t* bme, BME280_SAMPLE_t value);
BME280_RET_t bme280_set_p_sample(BME280_t* bme, BME280_SAMPLE_t value);
BME280_RET_t bme280_set_h_sample(BME280_t* bme, BME280_SAMPLE_t value);
BME280_RET_t bme280_set_standby(BME280_t* bme, BME280_STANDBY_t value);
BME280_RET_t bme280_set_filter(BME280_t* bme, BME280_FILTER_t value);
/* Sensor readings */
BME280_RET_t bme280_get_temperature(BME280_t* bme, float* value);
BME280_RET_t bme280_get_pressure(BME280_t* bme, float* value);
BME280_RET_t bme280_get_humidity(BME280_t* bme, float* value);
BME280_RET_t bme280_get_dewpoint(BME280_t* bme, float* value);


#endif // _ASNX_BME280_H_
