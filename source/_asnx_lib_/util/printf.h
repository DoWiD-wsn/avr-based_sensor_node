/*****
 * @brief   ASN(x) printf library
 *
 * Library to enable printf functionality.
 *
 * @file    /_asnx_lib_/util/printf.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.1.0 $
 * @date    $Date: 2021/05/10 $
 *
 * @see     https://github.com/mpaland/printf
 *****/

#ifndef _ASNX_PRINTF_H_
#define _ASNX_PRINTF_H_

/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>


/***** MACROS *********************************************************/
/* 'ntoa' conversion buffer size (default: 32 byte) */
#define PRINTF_NTOA_BUFFER_SIZE         (32U)
/* 'ftoa' conversion buffer size (default: 32 byte) */
#define PRINTF_FTOA_BUFFER_SIZE         (32U)
/* Define the default floating point precision (default: 6 digits) */
#define PRINTF_DEFAULT_FLOAT_PRECISION  (6U)
/* Define the largest float suitable to print with %f (default: 1e9) */
#define PRINTF_MAX_FLOAT                (1e9)
/* Internal flag definitions */
#define PRINTF_FLAGS_ZEROPAD            (1U <<  0U)
#define PRINTF_FLAGS_LEFT               (1U <<  1U)
#define PRINTF_FLAGS_PLUS               (1U <<  2U)
#define PRINTF_FLAGS_SPACE              (1U <<  3U)
#define PRINTF_FLAGS_HASH               (1U <<  4U)
#define PRINTF_FLAGS_UPPERCASE          (1U <<  5U)
#define PRINTF_FLAGS_CHAR               (1U <<  6U)
#define PRINTF_FLAGS_SHORT              (1U <<  7U)
#define PRINTF_FLAGS_LONG               (1U <<  8U)
#define PRINTF_FLAGS_LONG_LONG          (1U <<  9U)
#define PRINTF_FLAGS_PRECISION          (1U << 10U)
#define PRINTF_FLAGS_ADAPT_EXP          (1U << 11U)


/***** FUNCTION PROTOTYPES ********************************************/
void printf_init(void (*callback)(char c));
#define printf printf_
int printf_(const char* format, ...);
#define sprintf sprintf_
int sprintf_(char* buffer, const char* format, ...);
#define snprintf  snprintf_
int snprintf_(char* buffer, size_t count, const char* format, ...);
#define vsnprintf vsnprintf_
int vsnprintf_(char* buffer, size_t count, const char* format, va_list va);
#define vprintf vprintf_
int vprintf_(const char* format, va_list va);
int fctprintf(void (*out)(char character, void* arg), void* arg, const char* format, ...);


#endif // _ASNX_PRINTF_H_
