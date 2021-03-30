/**
 *  Header file for BME280 sensor.
 * 
 *  https://github.com/rm-hull/bme280
 *  https://github.com/bitbank2/bme280
 *  https://github.com/BoschSensortec/BME280_driver
 */

#ifndef _SENS_BME280_H_
#define _SENS_BME280_H_

/***** INCLUDES ***************************************************************/
#include <stdio.h>
#include <stdint.h>


/***** MACROS *****************************************************************/
/*** I2C specific ***/
#define BME280_I2C_ADD_BASE             (0x76)
/*** Misc ***/
#define BME280_READ_TIMEOUT             (500)
/*** Reset value ***/
#define BME280_RESET_VALUE              (0xB6)
/*** Register Values ***/
/* REGISTER ADDRESSES */
#define BME280_REG_CHIPID               (0xD0)
#define BME280_REG_RESET                (0xE0)
#define BME280_REG_CTRL_HUM             (0xF2)
#define BME280_REG_STATUS               (0xF3)
#define BME280_REG_CTRL_MEAS            (0xF4)
#define BME280_REG_CONFIG               (0xF5)
#define BME280_REG_P_MSB                (0xF7)
#define BME280_REG_P_LSB                (0xF8)
#define BME280_REG_P_XLSB               (0xF9)
#define BME280_REG_T_MSB                (0xFA)
#define BME280_REG_T_LSB                (0xFB)
#define BME280_REG_T_XLSB               (0xFC)
#define BME280_REG_H_MSB                (0xFD)
#define BME280_REG_H_LSB                (0xFE)

/* STATUS BITS */
#define BME280_STATUS_MEASURING         (0x08)
#define BME280_STATUS_UPDATING          (0x01)
/* TEMPERATURE */
#define BME280_REG_DIG_T1               (0x88)
#define BME280_REG_DIG_T2               (0x8A)
#define BME280_REG_DIG_T3               (0x8C)
/* PRESSURE */
#define BME280_REG_DIG_P1               (0x8E)
#define BME280_REG_DIG_P2               (0x90)
#define BME280_REG_DIG_P3               (0x92)
#define BME280_REG_DIG_P4               (0x94)
#define BME280_REG_DIG_P5               (0x96)
#define BME280_REG_DIG_P6               (0x98)
#define BME280_REG_DIG_P7               (0x9A)
#define BME280_REG_DIG_P8               (0x9C)
#define BME280_REG_DIG_P9               (0x9E)
/* HUMIDITY */
#define BME280_REG_DIG_H1               (0xA1)
#define BME280_REG_DIG_H2               (0xE1)
#define BME280_REG_DIG_H3               (0xE3)
#define BME280_REG_DIG_H4               (0xE4)
#define BME280_REG_DIG_H5               (0xE5)
#define BME280_REG_DIG_H6               (0xE6)
#define BME280_REG_DIG_H7               (0xE7)


/***** ENUMERATION ************************************************************/
/*** GENERAL ***/
/* Function return values */
typedef enum {
                BME280_RET_ERROR = -1,
                BME280_RET_OK = 0
             } bme280_ret_t;
/*** SETTINGS ***/
/* SAMPLING MODES */
typedef enum {
                BME280_SAMPLE_NONE  = 0,
                BME280_SAMPLE_1     = 1,
                BME280_SAMPLE_2     = 2,
                BME280_SAMPLE_4     = 3,
                BME280_SAMPLE_8     = 4,
                BME280_SAMPLE_16    = 5
             } bme280_sample_t;
/* OPERATING MODES */
typedef enum {
                BME280_MODE_SLEEP   = 0,
                BME280_MODE_FORCED  = 1,
                BME280_MODE_NORMAL  = 2
             } bme280_mode_t;
/* STANDBY SETTINGS */
typedef enum {
                BME280_STANDBY_0_5  = 0,
                BME280_STANDBY_62_5 = 1,
                BME280_STANDBY_125  = 2,
                BME280_STANDBY_250  = 3,
                BME280_STANDBY_500  = 4,
                BME280_STANDBY_1000 = 5,
                BME280_STANDBY_10   = 6,
                BME280_STANDBY_20   = 7
             } bme280_standby_t;
/* FILTER SETTINGS */
typedef enum {
                BME280_FILTER_OFF   = 0,
                BME280_FILTER_2     = 1,
                BME280_FILTER_4     = 2,
                BME280_FILTER_8     = 3,
                BME280_FILTER_16    = 4
             } bme280_filter_t;
/* SPI SETTINGS */
typedef enum {
                BME280_SPI_OFF      = 0,
                BME280_SPI_ON       = 1
             } bme280_spi_t;


/***** STRUCTURES *************************************************************/
/* Sensor calibration structure */
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
} bme280_calib_t;

/* Sensor configuration structure */
typedef struct {
    uint8_t address;        // I2C address
    uint8_t mode;           // operating mode
    uint8_t t_sample;       // temperature sampling mode
    uint8_t p_sample;       // pressure sampling mode
    uint8_t h_sample;       // humidity sampling mode
    uint8_t standby;        // standby mode
    uint8_t filter;         // filter mode
    int32_t t_fine;         // temperature fine reading
    bme280_calib_t calib;   // sensor calibration values
} bme280_t;


/***** FUNCTION PROTOTYPES ****************************************************/
/* General */
bme280_ret_t bme280_init(bme280_t* bme);
/* Configuration - read */
bme280_ret_t bme280_get_chipid(bme280_t* bme, uint8_t* value);
bme280_ret_t bme280_get_ctrl_hum(bme280_t* bme, uint8_t* value);
bme280_ret_t bme280_get_status(bme280_t* bme, uint8_t* value);
bme280_ret_t bme280_get_ctrl_meas(bme280_t* bme, uint8_t* value);
bme280_ret_t bme280_get_config(bme280_t* bme, uint8_t* value);
/* Configuration  - write */
bme280_ret_t bme280_reset(bme280_t* bme);
bme280_ret_t bme280_set_mode(bme280_t* bme, bme280_mode_t value);
bme280_ret_t bme280_set_t_sample(bme280_t* bme, bme280_sample_t value);
bme280_ret_t bme280_set_p_sample(bme280_t* bme, bme280_sample_t value);
bme280_ret_t bme280_set_h_sample(bme280_t* bme, bme280_sample_t value);
bme280_ret_t bme280_set_standby(bme280_t* bme, bme280_standby_t value);
bme280_ret_t bme280_set_filter(bme280_t* bme, bme280_filter_t value);
bme280_ret_t bme280_spi_enable(bme280_t* bme);
bme280_ret_t bme280_spi_disable(bme280_t* bme);
/* Sensor readings */
bme280_ret_t bme280_get_temperature(bme280_t* bme, float* value);
bme280_ret_t bme280_get_pressure(bme280_t* bme, float* value);
bme280_ret_t bme280_get_humidity(bme280_t* bme, float* value);
bme280_ret_t bme280_get_dewpoint(bme280_t* bme, float* value);


#endif // _SENS_BME280_H_
