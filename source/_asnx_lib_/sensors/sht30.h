/*!
 * @brief   ASN(x) SHT30 sensor library -- header file
 *
 * Library to support the SHT30 sensor (via I2C).
 *
 * @file    /_asnx_lib_/sensors/sht30.h
 * @author  Dominik Widhalm
 * @version 1.0.0
 * @date    2022/04/13
 */

#ifndef _ASNX_SHT30_H_
#define _ASNX_SHT30_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
/*** AVR ***/
#include <util/delay.h>
/*** ASNX LIB ***/
#include "i2c/i2c.h"


/***** DEFINES ********************************************************/
/*** I2C General ***/
/*! I2C default address */
#define SHT30_I2C_ADDRESS               0x44

/*** Sensor specific ***/
/*! Delay after requesting the sensor to wake-up [us] */
#define SHT03_WAKEUP_DELAY              250
/*! Delay for measurement poling [ms] */
#define SHT30_POLLING_DELAY             1
/*! Maximum number of retries for polling */
#define SHT30_POLLING_MAX               20
/*! CRC polynomial: P(x) = x^8 + x^5 + x^4 + 1 = 100110001 */
#define SHT30_CRC_POLYNOMIAL            0x131

/*** Measurement setting ***/
/*! Measure precision/repeatability (L=0; M=1; H=2; default: 0) */
#define SHT30_MEASURE_REPEATABILITY     0
/*! Use clock stretching during one-time measurement (default: 0) */
#define SHT30_CLOCK_STRETCHING          0
/*! Measurements per second for periodic mode (0.5=0; 1=1; 2=2; 4=3; 10=4; default: 0) */
#define SHT30_MEASUREMENTS_PER_SECOND   0

/*** I2C commands ***/
/*! Readout of measurement results for periodic mode */
#define SHT30_COM_FETCH                 0xE000
/*! Periodic measurement with ART */
#define SHT30_COM_ART                   0x2B32
/*! Stop periodic data acquisition mode with BREAK */
#define SHT30_COM_BREAK                 0x3093
/*! Soft reset */
#define SHT30_COM_SOFT_RESET            0x30A2
/*! Heater on (for testing purposes) */
#define SHT30_COM_HEATER_ENABLE         0x306D
/*! Heater off (for testing purposes) */
#define SHT30_COM_HEATER_DISABLE        0x3066
/*! Status register */
#define SHT30_COM_STATUS                0xF32D
/*! Clear status register */
#define SHT30_COM_CLEAR_STATUS          0x3041

/*** I2C Status register bit fields ***/
/*! Write data checksum status */
#define SHT30_COM_STATUS_CRC            0
/*! Command status */
#define SHT30_COM_STATUS_COM            1
/*! System reset detected */
#define SHT30_COM_STATUS_RST            4
/*! T tracking alert */
#define SHT30_COM_STATUS_T_ALERT        10
/*! H tracking alert */
#define SHT30_COM_STATUS_H_ALERT        11
/*! Heater status */
#define SHT30_COM_STATUS_HEATER         13
/*! Alert pending status */
#define SHT30_COM_STATUS_ALERT_PEDNING  15

/*** Measurement commands ***/
// Single shot mode
/// Clock stretching
/*! Measure with clock stretching (high repeatability) */
#define SHT30_COM_MEAS_OT_CST_H         0x2C06
/*! Measure with clock stretching (medium repeatability) */
#define SHT30_COM_MEAS_OT_CST_M         0x2C0D
/*! Measure with clock stretching (low repeatability) */
#define SHT30_COM_MEAS_OT_CST_L         0x2C10
/// Polling
/*! Measure with polling (high repeatability) */
#define SHT30_COM_MEAS_OT_POL_H         0x2400
/*! Measure with polling (medium repeatability) */
#define SHT30_COM_MEAS_OT_POL_M         0x240B
/*! Measure with polling (low repeatability) */
#define SHT30_COM_MEAS_OT_POL_L         0x2416
// Periodic mode
/// 0.5 mps
/*! Measure with 0.5 mps (high repeatability) */
#define SHT30_COM_MEAS_PER_0_5_H        0x2032
/*! Measure with 0.5 mps (medium repeatability) */
#define SHT30_COM_MEAS_PER_0_5_M        0x2024
/*! Measure with 0.5 mps (low repeatability) */
#define SHT30_COM_MEAS_PER_0_5_L        0x202F
/// 1 mps
/*! Measure with 1 mps (high repeatability) */
#define SHT30_COM_MEAS_PER_1_H          0x2130
/*! Measure with 1 mps (medium repeatability) */
#define SHT30_COM_MEAS_PER_1_M          0x2126
/*! Measure with 1 mps (low repeatability) */
#define SHT30_COM_MEAS_PER_1_L          0x212D
/// 2 mps
/*! Measure with 2 mps (high repeatability) */
#define SHT30_COM_MEAS_PER_2_H          0x2236
/*! Measure with 2 mps (medium repeatability) */
#define SHT30_COM_MEAS_PER_2_M          0x2220
/*! Measure with 2 mps (low repeatability) */
#define SHT30_COM_MEAS_PER_2_L          0x222B
/// 4 mps
/*! Measure with 4 mps (high repeatability) */
#define SHT30_COM_MEAS_PER_4_H          0x2334
/*! Measure with 4 mps (medium repeatability) */
#define SHT30_COM_MEAS_PER_4_M          0x2322
/*! Measure with 4 mps (low repeatability) */
#define SHT30_COM_MEAS_PER_4_L          0x2329
/// 10 mps
/*! Measure with 10 mps (high repeatability) */
#define SHT30_COM_MEAS_PER_10_H         0x2737
/*! Measure with 10 mps (medium repeatability) */
#define SHT30_COM_MEAS_PER_10_M         0x2721
/*! Measure with 10 mps (low repeatability) */
#define SHT30_COM_MEAS_PER_10_L         0x272A

/*! Actual one-time read command depends on settings */
#if SHT30_CLOCK_STRETCHING==0
#  if SHT30_MEASURE_REPEATABILITY==0
#    define SHT30_COM_MEASURE     SHT30_COM_MEAS_OT_POL_L
#  elif SHT30_MEASURE_REPEATABILITY==1
#    define SHT30_COM_MEASURE     SHT30_COM_MEAS_OT_POL_M
#  else
#    define SHT30_COM_MEASURE     SHT30_COM_MEAS_OT_POL_H
#  endif
#else
#  if SHT30_MEASURE_REPEATABILITY==0
#    define SHT30_COM_MEASURE     SHT30_COM_MEAS_OT_CST_L
#  elif SHT30_MEASURE_REPEATABILITY==1
#    define SHT30_COM_MEASURE     SHT30_COM_MEAS_OT_CST_M
#  else
#    define SHT30_COM_MEASURE     SHT30_COM_MEAS_OT_CST_H
#  endif
#endif

/*! Configuration of periodic mode depends on settings */
#if SHT30_MEASUREMENTS_PER_SECOND==0
#  if SHT30_MEASURE_REPEATABILITY==0
#    define SHT30_COM_PERIODIC    SHT30_COM_MEAS_PER_0_5_L
#  elif SHT30_MEASURE_REPEATABILITY==1
#    define SHT30_COM_PERIODIC    SHT30_COM_MEAS_PER_0_5_M
#  else
#    define SHT30_COM_PERIODIC    SHT30_COM_MEAS_PER_0_5_H
#  endif
#elif SHT30_MEASUREMENTS_PER_SECOND==1
#  if SHT30_MEASURE_REPEATABILITY==0
#    define SHT30_COM_PERIODIC    SHT30_COM_MEAS_PER_1_L
#  elif SHT30_MEASURE_REPEATABILITY==1
#    define SHT30_COM_PERIODIC    SHT30_COM_MEAS_PER_1_M
#  else
#    define SHT30_COM_PERIODIC    SHT30_COM_MEAS_PER_1_H
#  endif
#elif SHT30_MEASUREMENTS_PER_SECOND==2
#  if SHT30_MEASURE_REPEATABILITY==0
#    define SHT30_COM_PERIODIC    SHT30_COM_MEAS_PER_2_L
#  elif SHT30_MEASURE_REPEATABILITY==1
#    define SHT30_COM_PERIODIC    SHT30_COM_MEAS_PER_2_M
#  else
#    define SHT30_COM_PERIODIC    SHT30_COM_MEAS_PER_2_H
#  endif
#elif SHT30_MEASUREMENTS_PER_SECOND==3
#  if SHT30_MEASURE_REPEATABILITY==0
#    define SHT30_COM_PERIODIC    SHT30_COM_MEAS_PER_4_L
#  elif SHT30_MEASURE_REPEATABILITY==1
#    define SHT30_COM_PERIODIC    SHT30_COM_MEAS_PER_4_M
#  else
#    define SHT30_COM_PERIODIC    SHT30_COM_MEAS_PER_4_H
#  endif
#else
#  if SHT30_MEASURE_REPEATABILITY==0
#    define SHT30_COM_PERIODIC    SHT30_COM_MEAS_PER_10_L
#  elif SHT30_MEASURE_REPEATABILITY==1
#    define SHT30_COM_PERIODIC    SHT30_COM_MEAS_PER_10_M
#  else
#    define SHT30_COM_PERIODIC    SHT30_COM_MEAS_PER_10_H
#  endif
#endif

/*** SHT30 function return values ***/
#define SHT30_RET_ERROR_CRC         (-3)
#define SHT30_RET_ERROR_NODEV       (-2)
#define SHT30_RET_ERROR             (-1)
#define SHT30_RET_OK                (0)


/***** STRUCTURES *****************************************************/
/*!
 * A structure to store the SHT30 module properties.
 */
typedef struct {
    uint8_t address;    /**< Sensors I2C address */
    uint8_t running;    /**< Sensor running in periodic mode */
} SHT30_t;


/***** FUNCTION PROTOTYPES ********************************************/
int8_t sht30_init(SHT30_t* dev, uint8_t address);
/* General */
int8_t sht30_reset(SHT30_t* dev);
int8_t sht30_heater_enable(SHT30_t* dev);
int8_t sht30_heater_disable(SHT30_t* dev);
int8_t sht30_get_status(SHT30_t* dev, uint16_t* status);
int8_t sht30_clr_status(SHT30_t* dev);
/* One-time mode */
int8_t sht30_get_temperature(SHT30_t* dev, float* temperature);
int8_t sht30_get_humidity(SHT30_t* dev, float* humidity);
int8_t sht30_get_temperature_humidity(SHT30_t* dev, float* temperature, float* humidity);
/* Periodic mode */
int8_t sht30_periodic_start(SHT30_t* dev);
int8_t sht30_periodic_break(SHT30_t* dev);
int8_t sht30_fetch_temperature(SHT30_t* dev, float* temperature);
int8_t sht30_fetch_humidity(SHT30_t* dev, float* humidity);
int8_t sht30_fetch_temperature_humidity(SHT30_t* dev, float* temperature, float* humidity);

#endif // _ASNX_SHT30_H_
