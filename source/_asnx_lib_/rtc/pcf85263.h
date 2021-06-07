/*!
 * @brief   ASN(x) PCF85263 RTC library -- header file
 *
 * Library to support the PCF85263 RTC module.
 *
 * @file    /_asnx_lib_/rtc/pcf85263.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1.0 $
 * @date    $Date: 2021/05/12 $
 */

#ifndef _ASNX_PCF85263_H_
#define _ASNX_PCF85263_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>


/***** DEFINES ********************************************************/
/* I2C address */
#define PCF85263_I2C_ADDRESS                0x51

/* Enable 100th seconds (0 .. disabled / 1 .. enabled) */
#define PCF85263_100TH_SECONDS_ENABLE       1
/* Use 12h (0) or 24h (1) mode */
#define PCF85263_24H_MODE_ENABLE            1

/* TS signal pin (default) */
#define PCF85263_TS_DDR                     (DDRB)
#define PCF85263_TS_PORT                    (PORTB)
#define PCF85263_TS_PIN                     (PINB)
#define PCF85263_TS_GPIO                    (PB0)

/*** RTC mode time registers (RTCM = 0) ***/
/* RTC time and date registers */
#define PCF85263_RTC_100TH_SECONDS          0x00        /**< RTC 100ths of seconds register (0-99 BCD) */
#define PCF85263_RTC_SECONDS                0x01        /**< RTC seconds register */
#define PCF85263_RTC_MINUTES                0x02        /**< RTC minutes register */
#define PCF85263_RTC_HOURS                  0x03        /**< RTC hours register */
#define PCF85263_RTC_DAYS                   0x04        /**< RTC days register */
#define PCF85263_RTC_WEEKDAYS               0x05        /**< RTC day-of-week register */
#define PCF85263_RTC_MONTHS                 0x06        /**< RTC month register */
#define PCF85263_RTC_YEARS                  0x07        /**< RTC year register (0-99 BCD) */
/* RTC alarm1 */
#define PCF85263_RTC_SECOND_ALARM1          0x08        /**< RTC alarm1 seconds register */
#define PCF85263_RTC_MINUTE_ALARM1          0x09        /**< RTC alarm1 minutes register */
#define PCF85263_RTC_HOUR_ALARM1            0x0A        /**< RTC alarm1 hours register */
#define PCF85263_RTC_DAY_ALARM1             0x0B        /**< RTC alarm1 days register */
#define PCF85263_RTC_MONTH_ALARM1           0x0C        /**< RTC alarm1 month register */
/* RTC alarm2 */
#define PCF85263_RTC_MINUTE_ALARM2          0x0D        /**< RTC alarm2 minutes register */
#define PCF85263_RTC_HOUR_ALARM2            0x0E        /**< RTC alarm2 minutes register */
#define PCF85263_RTC_WEEKDAY_ALARM          0x0F        /**< RTC alarm2 day-of-week register */
/* RTC alarm enables */
#define PCF85263_RTC_ALARM_ENABLES          0x10        /**< RTC alarm enables register */
#define PCF85263_RTC_ALARM_SEC_A1E          0x01        /**< Second alarm1 enable */
#define PCF85263_RTC_ALARM_MIN_A1E          0x02        /**< Minute alarm1 enable */
#define PCF85263_RTC_ALARM_HR_A1E           0x04        /**< Hour alarm1 enable */
#define PCF85263_RTC_ALARM_DAY_A1E          0x08        /**< Day alarm1 enable */
#define PCF85263_RTC_ALARM_MON_A1E          0x10        /**< Month alarm1 enable */
#define PCF85263_RTC_ALARM_MIN_A2E          0x20        /**< Minute alarm2 enable */
#define PCF85263_RTC_ALARM_HR_A2E           0x40        /**< Hour alarm2 enable */
#define PCF85263_RTC_ALARM_WDAY_A2E         0x80        /**< Day-of-week alarm2 enable */
/* RTC timestamp1 (TSR1) */
#define PCF85263_RTC_TSR1_SECONDS           0x11        /**< TSR1 seconds register */
#define PCF85263_RTC_TSR1_MINUTES           0x12        /**< TSR1 minutes register */
#define PCF85263_RTC_TSR1_HOURS             0x13        /**< TSR1 hours register */
#define PCF85263_RTC_TSR1_DAYS              0x14        /**< TSR1 days register */
#define PCF85263_RTC_TSR1_MONTHS            0x15        /**< TSR1 month register */
#define PCF85263_RTC_TSR1_YEARS             0x16        /**< TSR1 year register (0-99 BCD) */
/* RTC timestamp2 (TSR2) */
#define PCF85263_RTC_TSR2_SECONDS           0x17        /**< TSR2 seconds register */
#define PCF85263_RTC_TSR2_MINUTES           0x18        /**< TSR2 minutes register */
#define PCF85263_RTC_TSR2_HOURS             0x19        /**< TSR2 hours register */
#define PCF85263_RTC_TSR2_DAYS              0x1A        /**< TSR2 days register */
#define PCF85263_RTC_TSR2_MONTHS            0x1B        /**< TSR2 month register */
#define PCF85263_RTC_TSR2_YEARS             0x1C        /**< TSR2 year register (0-99 BCD) */
/* RTC timestamp3 (TSR3) */
#define PCF85263_RTC_TSR3_SECONDS           0x1D        /**< TSR3 seconds register */
#define PCF85263_RTC_TSR3_MINUTES           0x1E        /**< TSR3 minutes register */
#define PCF85263_RTC_TSR3_HOURS             0x1F        /**< TSR3 hours register */
#define PCF85263_RTC_TSR3_DAYS              0x20        /**< TSR3 days register */
#define PCF85263_RTC_TSR3_MONTHS            0x21        /**< TSR3 month register */
#define PCF85263_RTC_TSR3_YEARS             0x22        /**< TSR3 year register (0-99 BCD) */
/* RTC timestamp mode control register */
#define PCF85263_RTC_TSR_MODE               0x23        /**< Timestamp mode control register */

/*** Stop-watch mode time registers (RTCM = 1) ***/
/* Stop-watch time registers */
#define PCF85263_STW_100TH_SECONDS          0x00        /**< Stopwatch 100ths of seconds register (0-99 BCD) */
#define PCF85263_STW_SECONDS                0x01        /**< Stopwatch seconds register (0-59 BCD) */
#define PCF85263_STW_MINUTES                0x02        /**< Stopwatch minutes register (0-59 BCD) */
#define PCF85263_STW_HOURS_XX_XX_00         0x03        /**< Stopwatch hours register xx_xx_00 (0-99 BCD) */
#define PCF85263_STW_HOURS_XX_00_XX         0x04        /**< Stopwatch hours register xx_00_xx (0-99 BCD) */
#define PCF85263_STW_HOURS_00_XX_XX         0x05        /**< Stopwatch hours register 00_xx_xx (0-99 BCD) */
/* Stop-watch alarm1 */
#define PCF85263_STW_SECOND_ALM1            0x08        /**< Stopwatch alarm1 seconds register (0-59 BCD) */
#define PCF85263_STW_MINUTE_ALM1            0x09        /**< Stopwatch alarm1 minutes register (0-59 BCD) */
#define PCF85263_STW_HR_XX_XX_00_ALM1       0x0A        /**< Stopwatch alarm1 hours register xx_xx_00 (0-99 BCD) */
#define PCF85263_STW_HR_XX_00_XX_ALM1       0x0B        /**< Stopwatch alarm1 hours register xx_00_xx (0-99 BCD) */
#define PCF85263_STW_HR_00_XX_XX_ALM1       0x0C        /**< Stopwatch alarm1 hours register 00_xx_xx (0-99 BCD) */
/* Stop-watch alarm2 */
#define PCF85263_STW_MINUTE_ALM2            0x0D        /**< Stopwatch alarm2 minutes register */
#define PCF85263_STW_HR_XX_00_ALM2          0x0E        /**< Stopwatch alarm2 hours register xx_00_xx (0-99 BCD) */
#define PCF85263_STW_HR_00_XX_ALM2          0x0F        /**< Stopwatch alarm2 hours register 00_xx_xx (0-99 BCD) */
/* Stop-watch alarm enables */
#define PCF85263_STW_ALARM_ENABLES          0x10        /**< Stopwatch alarm enable control register */
#define PCF85263_STW_SEC_A1E                0x01        /**< Bit 0: second alarm1 enable */
#define PCF85263_STW_MIN_A1E                0x02        /**< Bit 1: minute alarm1 enable */
#define PCF85263_STW_HR_XX_XX_00_A1E        0x04        /**< Bit 2: hour alarm1 enable */
#define PCF85263_STW_HR_XX_00_XX_A1E        0x08        /**< Bit 3: hundreds of hours alarm1 enable */
#define PCF85263_STW_HR_00_XX_XX_A1E        0x10        /**< Bit 4: 10-thousands of hours alarm1 enable */
#define PCF85263_STW_MIN_A2E                0x20        /**< Bit 5: minute alarm2 enable */
#define PCF85263_STW_HR_XX_00_A2E           0x40        /**< Bit 6: tens of hours alarm2 enable */
#define PCF85263_STW_HR_00_XX_A2E           0x80        /**< Bit 7: thousands of hours alarm2 enable */
/* Stop-watch timestamp1 (TSR1) */
#define PCF85263_STW_TSR1_SECONDS           0x11        /**< Stopwatch TSR1 seconds register (0-59 BCD) */
#define PCF85263_STW_TSR1_MINUTES           0x12        /**< Stopwatch TSR1 minutes register (0-59 BCD) */
#define PCF85263_STW_TSR1_HR_XX_XX_00       0x13        /**< Stopwatch TSR1 hours register xx_xx_00 (0-99 BCD) */
#define PCF85263_STW_TSR1_HR_XX_00_XX       0x14        /**< Stopwatch TSR1 hours register xx_00_xx (0-99 BCD) */
#define PCF85263_STW_TSR1_HR_00_XX_XX       0x15        /**< Stopwatch TSR1 hours register 00_xx_xx (0-99 BCD) */
/* Stop-watch timestamp2 (TSR2) */
#define PCF85263_STW_TSR2_SECONDS           0x17        /**< Stopwatch TSR2 seconds register (0-59 BCD) */
#define PCF85263_STW_TSR2_MINUTES           0x18        /**< Stopwatch TSR2 minutes register (0-59 BCD) */
#define PCF85263_STW_TSR2_HR_XX_XX_00       0x19        /**< Stopwatch TSR2 hours register xx_xx_00 (0-99 BCD) */
#define PCF85263_STW_TSR2_HR_XX_00_XX       0x1A        /**< Stopwatch TSR2 hours register xx_00_xx (0-99 BCD) */
#define PCF85263_STW_TSR2_HR_00_XX_XX       0x1B        /**< Stopwatch TSR2 hours register 00_xx_xx (0-99 BCD) */
/* Stop-watch timestamp3 (TSR3) */
#define PCF85263_STW_TSR3_SECONDS           0x1D        /**< Stopwatch TSR3 seconds register (0-59 BCD) */
#define PCF85263_STW_TSR3_MINUTES           0x1E        /**< Stopwatch TSR3 minutes register (0-59 BCD) */
#define PCF85263_STW_TSR3_HR_XX_XX_00       0x1F        /**< Stopwatch TSR3 hours register xx_xx_00 (0-99 BCD) */
#define PCF85263_STW_TSR3_HR_XX_00_XX       0x20        /**< Stopwatch TSR3 hours register xx_00_xx (0-99 BCD) */
#define PCF85263_STW_TSR3_HR_00_XX_XX       0x21        /**< Stopwatch TSR3 hours register 00_xx_xx (0-99 BCD) */
/* Stop-watch timestamp mode control */
#define PCF85263_STW_TSR_MODE               0x23        /**< Stopwatch timestamp mode control register */

/*** Control and function registers ***/
/* Offset register */
#define PCF85263_CTL_OFFSET                 0x24        /**< Offset register */
/* Oscillator control registers */
#define PCF85263_CTL_OSCILLATOR             0x25        /**< Oscillator control register */
#define PCF85263_CTL_OSC_CL_OFFSET          0           /**< CL bit offset */
#define PCF85263_CTL_OSC_CL_MASK            0x03        /**< Bits 0-1: quartz oscillator load capacitance */
#  define PCF85263_CTL_OSC_CL_7PF           0x00        /**< Load capacitance 7.0 pF */
#  define PCF85263_CTL_OSC_CL_6PF           0x01        /**< Load capacitance 6.0 pF */
#  define PCF85263_CTL_OSC_CL_12p5PF        0x02        /**< Load capacitance 12.5 pF */
#define PCF85263_CTL_OSC_OSCD_OFFSET        2           /**< OSCD bit offset */
#define PCF85263_CTL_OSC_OSCD_MASK          0xC0        /**< Bits 2-3: oscillator driver bits */
#  define PCF85263_CTL_OSC_OSCD_NORMAL      0x00        /**< Normal drive; RS(max): 100 kohm */
#  define PCF85263_CTL_OSC_OSCD_LOW         0x04        /**< Low drive; RS(max): 60 kohm; reduced IDD */
#  define PCF85263_CTL_OSC_OSCD_HIGH        0x08        /**< High drive; RS(max): 500 kohm; increased IDD */
#define PCF85263_CTL_OSC_LOWJ               0x10        /**< Bit 4: low jitter mode */
#define PCF85263_CTL_OSC_12_24              0x20        /**< Bit 5: 12-/24-hour mode */
#define PCF85263_CTL_OSC_OFFM               0x40        /**< Bit 6: offset calibration mode */
#define PCF85263_CTL_OSC_CLKIV              0x80        /**< Bit 7: output clock inversion */
/* Battery switch control registers */
#define PCF85263_CTL_BATTERY_SWITCH         0x26        /**< Battery switch control register */
#define PCF85263_CTL_BATTERY_BSTH           0x01        /**< Bit 0: threshold voltage control */
#define PCF85263_CTL_BATTERY_BSM_OFFSET     1           /**< BSM bits offset */
#define PCF85263_CTL_BATTERY_BSM_MASK       0x06        /**< Bits 1-2: battery switch mode bits */
#  define PCF85263_CTL_BATTERY_BSM_VTH      0x00        /**< Switching at the Vth level */
#  define PCF85263_CTL_BATTERY_BSM_VBAT     0x02        /**< Switching at the VBAT level */
#  define PCF85263_CTL_BATTERY_BSM_MAX      0x04        /**< Switching at the higher level of Vth or VBAT */
#  define PCF85263_CTL_BATTERY_BSM_MIN      0x06        /**< Switching at the lower level of Vth or VBAT */
#define PCF85263_CTL_BATTERY_BSRR           0x08        /**< Bit 3: battery switch refresh rate */
#define PCF85263_CTL_BATTERY_BSOFF          0x10        /**< Bit 4: battery switch on/off */
/* Pin IO control registers */
#define PCF85263_CTL_PIN_IO                 0x27        /**< Pin input/output control register */
#define PCF85263_CTL_INTAPM_OFFSET          0           /**< INTAPM bits offset */
#define PCF85263_CTL_INTAPM_MASK            0x03        /**< Bits 0-1: INTA pin mode */
#  define PCF85263_CTL_INTAPM_CLK           0x00        /**< CLK output mode */
#  define PCF85263_CTL_INTAPM_BAT           0x01        /**< Battery mode indication */
#  define PCF85263_CTL_INTAPM_INTA          0x02        /**< INTA output */
#  define PCF85263_CTL_INTAPM_HIZ           0x03        /**< Hi-Z */
#define PCF85263_CTL_TSPM_OFFSET            2           /**< TSPM bits offset */
#define PCF85263_CTL_TSPM_MASK              0x0C        /**< Bits 2-3: TS pin I/O control */
#  define PCF85263_CTL_TSPM_DISABLED        0x00        /**< Disabled; input can be left floating */
#  define PCF85263_CTL_TSPM_INTB            0x04        /**< INTB output; push-pull */
#  define PCF85263_CTL_TSPM_CLK             0x08        /**< CLK output; push-pull */
#  define PCF85263_CTL_TSPM_INPUT           0x0C        /**< Input mode */
#define PCF85263_CTL_TSIM                   0x10        /**< Bit 4: TS pin input mode */
#define PCF85263_CTL_TSL                    0x20        /**< Bit 5: TS pin input sense */
#define PCF85263_CTL_TSPULL                 0x40        /**< Bit 6: TS pin pull-up resistor value */
#define PCF85263_CTL_CLKPM                  0x80        /**< Bit 7: CLK pin mode */
/* Function control registers */
#define PCF85263_CTL_FUNCTION               0x28        /**< Function control register */
#define PCF85263_CTL_FUNC_COF_OFFSET        0           /**< COF bits offset */
#define PCF85263_CTL_FUNC_COF_MASK          0x07        /**< Bits 0-2: clock output frequency */
#  define PCF85263_CTL_FUNC_COF_32KHZ       0x00        /**< 32768Hz (60:40 to 40:60) */
#  define PCF85263_CTL_FUNC_COF_16KHZ       0x01        /**< 16384Hz (50:50) */
#  define PCF85263_CTL_FUNC_COF_8KHZ        0x02        /**< 8192Hz (50:50) */
#  define PCF85263_CTL_FUNC_COF_4KHZ        0x03        /**< 4096Hz (50:50) */
#  define PCF85263_CTL_FUNC_COF_2KHZ        0x04        /**< 2048Hz (50:50) */
#  define PCF85263_CTL_FUNC_COF_1KHZ        0x05        /**< 1024Hz (50:50) */
#  define PCF85263_CTL_FUNC_COF_1HZ         0x06        /**< 1Hz (50:50) */
#  define PCF85263_CTL_FUNC_COF_STATIC      0x07        /**< Static LOW */
#define PCF85263_CTL_FUNC_STOPM             0x08        /**< Bit 3: STOP mode */
#define PCF85263_CTL_FUNC_RTCM              0x10        /**< Bit 4: RTC mode */
#define PCF85263_CTL_FUNC_PI_OFFSET         5           /**< PI bits offset */
#define PCF85263_CTL_FUNC_PI_MASK           0x60        /**< Bits 5-6: periodic interrupt */
#  define PCF85263_CTL_FUNC_PI_NONE         0x00        /**< No periodic interrupt */
#  define PCF85263_CTL_FUNC_PI_SEC          0x20        /**< Once per second */
#  define PCF85263_CTL_FUNC_PI_MIN          0x40        /**< Once per minute */
#  define PCF85263_CTL_FUNC_PI_HOUR         0x60        /**< Once per hour */
#define PCF85263_CTL_FUNC_100TH             0x80        /**< Bit 7: 100th seconds mode */
/* INTA control registers */
#define PCF85263_CTL_INTA_ENABLE            0x29        /**< Interrupt A control register */
#define PCF85263_CTL_INTA_WDIEA             0x01        /**< Bit 0: watchdog interrupt enable */
#define PCF85263_CTL_INTA_BSIEA             0x02        /**< Bit 1: battery switch interrupt enable */
#define PCF85263_CTL_INTA_TSRIEA            0x04        /**< Bit 2: timestamp register interrupt enable */
#define PCF85263_CTL_INTA_A2IEA             0x08        /**< Bit 3: alarm2 interrupt enable */
#define PCF85263_CTL_INTA_A1IEA             0x10        /**< Bit 4: alarm1 interrupt enable */
#define PCF85263_CTL_INTA_OIEA              0x20        /**< Bit 5: offset correction interrupt enable */
#define PCF85263_CTL_INTA_PIEA              0x40        /**< Bit 6: periodic interrupt enable */
#define PCF85263_CTL_INTA_ILPA              0x80        /**< Bit 7: interrupt generates a pulse */
/* INTB control registers */
#define PCF85263_CTL_INTB_ENABLE            0x2A        /**< Interrupt B control register */
#define PCF85263_CTL_INTB_WDIEB             0x01        /**< Bit 0: watchdog interrupt enable */
#define PCF85263_CTL_INTB_BSIEB             0x02        /**< Bit 1: battery switch interrupt enable */
#define PCF85263_CTL_INTB_TSRIEB            0x04        /**< Bit 2: timestamp register interrupt enable */
#define PCF85263_CTL_INTB_A2IEB             0x08        /**< Bit 3: alarm2 interrupt enable */
#define PCF85263_CTL_INTB_A1IEB             0x10        /**< Bit 4: alarm1 interrupt enable */
#define PCF85263_CTL_INTB_OIEB              0x20        /**< Bit 5: offset correction interrupt enable */
#define PCF85263_CTL_INTB_PIEB              0x40        /**< Bit 6: periodic interrupt enable */
#define PCF85263_CTL_INTB_ILPB              0x80        /**< Bit 7: interrupt generates a pulse */
/* Flags control registers */
#define PCF85263_CTL_FLAGS                  0x2B        /**< Flag status register */
#define PCF85263_CTL_FLAGS_TSR1F            0x01        /**< Bit 0: timestamp register 1 event flag */
#define PCF85263_CTL_FLAGS_TSR2F            0x02        /**< Bit 1: timestamp register 2 event flag */
#define PCF85263_CTL_FLAGS_TSR3F            0x04        /**< Bit 2: timestamp register 3 event flag */
#define PCF85263_CTL_FLAGS_BSF              0x08        /**< Bit 3: battery switch flag */
#define PCF85263_CTL_FLAGS_WDF              0x10        /**< Bit 4: watchdog flag */
#define PCF85263_CTL_FLAGS_A1F              0x20        /**< Bit 5: alarm1 flag */
#define PCF85263_CTL_FLAGS_A2F              0x40        /**< Bit 6: alarm2 flag */
#define PCF85263_CTL_FLAGS_PIF              0x80        /**< Bit 7: periodic interrupt flag */
/* RAM byte */
#define PCF85263_CTL_RAM_BYTE               0x2C        /**< RAM byte register */
/* Watchdog registers */
#define PCF85263_CTL_WATCHDOG               0x2D        /**< Watchdog control and status register */
#define PCF85263_CTL_WDS_OFFSET             0           /**< WDS bits offset */
#define PCF85263_CTL_WDS_MASK               0x03        /**< Bits 0-1: watchdog step size (source clock) */
#  define PCF85263_CTL_WDS_4SEC             0x00        /**< 4 seconds (0.25 Hz) */
#  define PCF85263_CTL_WDS_1SEC             0x01        /**< 1 second (1 Hz) */
#  define PCF85263_CTL_WDS_250MSEC          0x02        /**< 1/4 second (4 Hz) */
#  define PCF85263_CTL_WDS_67MSEC           0x03        /**< 1/16 second (16 Hz) */
#define PCF85263_CTL_WDR_OFFSET             2           /**< WDR bits offset */
#define PCF85263_CTL_WDR_MASK               0x7C        /**< Bits 2-6: watchdog register bits (counter value) */
#define PCF85263_CTL_WDM                    0x80        /**< Bit 7: watchdog mode */
/* Stop */
#define PCF85263_CTL_STOP_ENABLE            0x2E        /**< Stop enable register */
#define PCF85263_CTL_START                  0x00        /**< Bit 0: stop bit -> 0 = RTC clock runs */
#define PCF85263_CTL_STOP                   0x01        /**< Bit 0: stop bit -> 1 = RTC clock is stopped */
/* Reset */
#define PCF85263_CTL_RESETS                 0x2F        /**< Software reset control register */
#define PCF85263_CTL_CTS                    0x01        /**< Bit 0: clear timestamp */
#define PCF85263_CTL_SR                     0x08        /**< Bit 3: software reset */
#define PCF85263_CTL_CPR                    0x80        /**< Bit 7: clear prescaler */
#define PCF85263_CTL_RESETS_BITS            0x24        /**< Fixed register bits (read as "1") */

/*** Register flags ***/
#define PCF85263_FLAG_OS                    0x80        /**< Bit 7: oscillator stop */
#define PCF85263_FLAG_EMON                  0x80        /**< Bit 7: event monitor */
/* TSR1 mode control */
#define PCF85263_TSR_TSR1M_MASK             0x03        /**< Bit 0-1: timestamp register 1 mode */
#define PCF85263_TSR_TSR1M_NONE             0x00        /**< TSR1: no timestamp */
#define PCF85263_TSR_TSR1M_FE               0x01        /**< TSR1: record first TS pin event (FE) */
#define PCF85263_TSR_TSR1M_LE               0x02        /**< TSR1: record last TS pin event (LE) */
/* TSR2 mode control */
#define PCF85263_TSR_TSR2M_MASK             0x1C        /**< Bit 2-4: timestamp register 2 mode */
#define PCF85263_TSR_TSR2M_NONE             0x00        /**< TSR2: no timestamp */
#define PCF85263_TSR_TSR2M_FB               0x04        /**< TSR2: record first time switch to battery event */
#define PCF85263_TSR_TSR2M_LB               0x08        /**< TSR2: record last time switch to battery event */
#define PCF85263_TSR_TSR2M_LV               0x0C        /**< TSR2: record last time switch to VDD event */
#define PCF85263_TSR_TSR2M_FE               0x10        /**< TSR2: record first TS pin event */
#define PCF85263_TSR_TSR2M_LE               0x14        /**< TSR2: record last TS pin event */
/* TSR3 mode control */
#define PCF85263_TSR_TSR3M_MASK             0xC0        /**< Bit 6-7: Timestamp register 3 mode */
#define PCF85263_TSR_TSR3M_NONE             0x00        /**< TSR3: no timestamp */
#define PCF85263_TSR_TSR3M_FB               0x40        /**< TSR3: record first time switch to battery event */
#define PCF85263_TSR_TSR3M_LB               0x80        /**< TSR3: record last time switch to battery event */
#define PCF85263_TSR_TSR3M_LV               0xC0        /**< TSR3: record Last time switch to VDD event */

/*** Time value register masks ***/
#define PCF85263_SECONDS_MASK               0x7F        /**< Bits 0-6: seconds (0-59 BCD) */
#define PCF85263_MINUTES_MASK               0x7F        /**< Bits 0-6: minutes (0-59 BCD) */
#define PCF85263_HOURS12_MASK               0x1F        /**< Bits 0-4: hours (1-12 BCD) in 12 hour mode */
#define PCF85263_HOURS12_AMPM               0x20        /**< Bit 5: AM/PM */
#define PCF85263_HOURS24_MASK               0x3F        /**< Bits 0-5: hours (0-23 BCD) in 24 hour mode */
#define PCF85263_DAYS_MASK                  0x3F        /**< Bits 0-5: days (1-31 BCD) */
#define PCF85263_WEEKDAYS_MASK              0x07        /**< Bits 0-2: day of the week (0-6) */
#define PCF85263_MONTHS_MASK                0x1F        /**< Bits 0-4: month (1-12 BCD) */


/***** ENUMERATION ****************************************************/
/* Enumeration for the PCR85263 function return values */
typedef enum {
    PCF85263_RET_NO_DEV     = -2,
    PCF85263_RET_ERROR      = -1,
    PCF85263_RET_OK         = 0
} PCF85263_RET_t;


/***** STRUCTURES *****************************************************/
/***
 * A structure to store date and time information.
 ***/
typedef struct {
#if PCF85263_100TH_SECONDS_ENABLE==1
    uint8_t msec10;     /**< 100th-seconds (0-99) */
#endif
    uint8_t seconds;    /**< Seconds (0-59) */
    uint8_t minutes;    /**< Minutes (0-59) */
#if PCF85263_24H_MODE_ENABLE==1
    uint8_t hours;      /**< Hours (0-23) */
#else
    uint8_t ampm;       /**< AM (0) or PM (1) */
    uint8_t hours;      /**< Hours (1-12) */
#endif
    uint8_t days;       /**< Days (1-31) */
    uint8_t wday;       /**< Day-of-week (0-6) */
    uint8_t months;     /**< Months (1-12) */
    uint8_t years;      /**< Years (0-99) */
} PCF85263_DATETIME_t;

/***
 * A structure to store stopwatch time information.
 ***/
typedef struct {
#if PCF85263_100TH_SECONDS_ENABLE==1
    uint8_t msec10;     /**< 100th-seconds (0-99) */
#endif
    uint8_t seconds;    /**< Seconds (0-59) */
    uint8_t minutes;    /**< Minutes (0-59) */
    uint32_t hours;     /**< Hours (0-999999) */
} PCF85263_CNTTIME_t;


/***** FUNCTION PROTOTYPES ********************************************/
/*** General ***/
/* Init */
PCF85263_RET_t pcf85263_init(void);
/* Datetime/time structure */
void pcf85263_clear_rtc_datetime(PCF85263_DATETIME_t* data);
void pcf85263_clear_stw_time(PCF85263_CNTTIME_t* data);
/* Read/write register */
PCF85263_RET_t pcf85263_read_reg(uint8_t reg, uint8_t* byte);
PCF85263_RET_t pcf85263_write_reg(uint8_t reg, uint8_t byte);
/* Start/stop */
PCF85263_RET_t pcf85263_start(void);
PCF85263_RET_t pcf85263_stop(void);
/* Reset */
PCF85263_RET_t pcf85263_reset(void);
PCF85263_RET_t pcf85263_reset_prescaler(void);
PCF85263_RET_t pcf85263_reset_timestamp(void);
/* EMON & OS monitor flags */
PCF85263_RET_t pcf85263_get_EMON(uint8_t* value);
PCF85263_RET_t pcf85263_get_OS(uint8_t* value);
PCF85263_RET_t pcf85263_clear_OS(void);

/*** Configuration ***/
/* Offset */
PCF85263_RET_t pcf85263_read_offset(uint8_t* value);
PCF85263_RET_t pcf85263_write_offset(uint8_t value);
/* Oscillator */
PCF85263_RET_t pcf85263_get_oscillator(uint8_t* value);
PCF85263_RET_t pcf85263_set_oscillator(uint8_t value);
/* Battery switch */
PCF85263_RET_t pcf85263_get_batteryswitch(uint8_t* value);
PCF85263_RET_t pcf85263_set_batteryswitch(uint8_t value);
/* Pin IO */
PCF85263_RET_t pcf85263_get_pin_io(uint8_t* value);
PCF85263_RET_t pcf85263_set_pin_io(uint8_t value);
/* Function */
PCF85263_RET_t pcf85263_get_function(uint8_t* value);
PCF85263_RET_t pcf85263_set_function(uint8_t value);
/* INTA enable */
PCF85263_RET_t pcf85263_get_inta_en(uint8_t* value);
PCF85263_RET_t pcf85263_set_inta_en(uint8_t value);
/* INTA enable */
PCF85263_RET_t pcf85263_get_intb_en(uint8_t* value);
PCF85263_RET_t pcf85263_set_intb_en(uint8_t value);
/* Flags */
PCF85263_RET_t pcf85263_get_flags(uint8_t* value);
PCF85263_RET_t pcf85263_set_flags(uint8_t value);
/* RAM byte */
PCF85263_RET_t pcf85263_read_ram(uint8_t* byte);
PCF85263_RET_t pcf85263_write_ram(uint8_t byte);
/* Watchdog */
PCF85263_RET_t pcf85263_get_watchdog(uint8_t* value);
PCF85263_RET_t pcf85263_set_watchdog(uint8_t value);

/*** Date/time ***/
/* Shared */
PCF85263_RET_t pcf85263_get_100th_seconds(uint8_t* seconds_100th);
PCF85263_RET_t pcf85263_set_100th_seconds(uint8_t seconds_100th);
PCF85263_RET_t pcf85263_get_seconds(uint8_t* seconds);
PCF85263_RET_t pcf85263_set_seconds(uint8_t seconds);
PCF85263_RET_t pcf85263_get_minutes(uint8_t* minutes);
PCF85263_RET_t pcf85263_set_minutes(uint8_t minutes);
/* RTC mode */
#if PCF85263_24H_MODE_ENABLE==1
PCF85263_RET_t pcf85263_get_hours(uint8_t* hours);
PCF85263_RET_t pcf85263_set_hours(uint8_t hours);
#else
PCF85263_RET_t pcf85263_get_hours(uint8_t* hours, uint8_t* ampm);
PCF85263_RET_t pcf85263_set_hours(uint8_t hours, uint8_t ampm);
#endif
PCF85263_RET_t pcf85263_get_days(uint8_t* days);
PCF85263_RET_t pcf85263_set_days(uint8_t days);
PCF85263_RET_t pcf85263_get_weekdays(uint8_t* weekdays);
PCF85263_RET_t pcf85263_set_weekdays(uint8_t weekdays);
PCF85263_RET_t pcf85263_get_months(uint8_t* months);
PCF85263_RET_t pcf85263_set_months(uint8_t months);
PCF85263_RET_t pcf85263_get_years(uint8_t* years);
PCF85263_RET_t pcf85263_set_years(uint8_t years);
PCF85263_RET_t pcf85263_get_rtc_datetime(PCF85263_DATETIME_t* data);
PCF85263_RET_t pcf85263_set_rtc_datetime(PCF85263_DATETIME_t* data);
/* Stop-watch mode */
PCF85263_RET_t pcf85263_get_hours_xx_xx_00(uint8_t* hours);
PCF85263_RET_t pcf85263_set_hours_xx_xx_00(uint8_t hours);
PCF85263_RET_t pcf85263_get_hours_xx_00_xx(uint8_t* hours);
PCF85263_RET_t pcf85263_set_hours_xx_00_xx(uint8_t hours);
PCF85263_RET_t pcf85263_get_hours_00_xx_xx(uint8_t* hours);
PCF85263_RET_t pcf85263_set_hours_00_xx_xx(uint8_t hours);
PCF85263_RET_t pcf85263_get_stw_time(PCF85263_CNTTIME_t* data);
PCF85263_RET_t pcf85263_set_stw_time(PCF85263_CNTTIME_t* data);

/*** Alarms ***/
/* RTC mode */
PCF85263_RET_t pcf85263_get_rtc_alarm_enables(uint8_t* value);
PCF85263_RET_t pcf85263_set_rtc_alarm_enables(uint8_t value);
PCF85263_RET_t pcf85263_get_rtc_alarm1(PCF85263_DATETIME_t* data);
PCF85263_RET_t pcf85263_set_rtc_alarm1(PCF85263_DATETIME_t* data);
PCF85263_RET_t pcf85263_get_rtc_alarm2(PCF85263_DATETIME_t* data);
PCF85263_RET_t pcf85263_set_rtc_alarm2(PCF85263_DATETIME_t* data);
/* Stop-watch mode */
PCF85263_RET_t pcf85263_get_stw_alarm_enables(uint8_t* value);
PCF85263_RET_t pcf85263_set_stw_alarm_enables(uint8_t value);
PCF85263_RET_t pcf85263_get_stw_alarm1(PCF85263_CNTTIME_t* data);
PCF85263_RET_t pcf85263_set_stw_alarm1(PCF85263_CNTTIME_t* data);
PCF85263_RET_t pcf85263_get_stw_alarm2(PCF85263_CNTTIME_t* data);
PCF85263_RET_t pcf85263_set_stw_alarm2(PCF85263_CNTTIME_t* data);

/*** Timestamps ***/
/* RTC mode */
PCF85263_RET_t pcf85263_get_rtc_timestamp_mode(uint8_t* value);
PCF85263_RET_t pcf85263_set_rtc_timestamp_mode(uint8_t value);
PCF85263_RET_t pcf85263_get_rtc_timestamp1(PCF85263_DATETIME_t* data);
PCF85263_RET_t pcf85263_set_rtc_timestamp1(PCF85263_DATETIME_t* data);
PCF85263_RET_t pcf85263_get_rtc_timestamp2(PCF85263_DATETIME_t* data);
PCF85263_RET_t pcf85263_set_rtc_timestamp2(PCF85263_DATETIME_t* data);
PCF85263_RET_t pcf85263_get_rtc_timestamp3(PCF85263_DATETIME_t* data);
PCF85263_RET_t pcf85263_set_rtc_timestamp3(PCF85263_DATETIME_t* data);
/* Stop-watch mode */
PCF85263_RET_t pcf85263_get_stw_timestamp_mode(uint8_t* value);
PCF85263_RET_t pcf85263_set_stw_timestamp_mode(uint8_t value);
PCF85263_RET_t pcf85263_get_stw_timestamp1(PCF85263_CNTTIME_t* data);
PCF85263_RET_t pcf85263_set_stw_timestamp1(PCF85263_CNTTIME_t* data);
PCF85263_RET_t pcf85263_get_stw_timestamp2(PCF85263_CNTTIME_t* data);
PCF85263_RET_t pcf85263_set_stw_timestamp2(PCF85263_CNTTIME_t* data);
PCF85263_RET_t pcf85263_get_stw_timestamp3(PCF85263_CNTTIME_t* data);
PCF85263_RET_t pcf85263_set_stw_timestamp3(PCF85263_CNTTIME_t* data);

#endif // _ASNX_PCF85263_H_
