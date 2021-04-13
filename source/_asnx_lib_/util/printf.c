/*****
 * @brief   ASN(x) printf library
 *
 * Library to enable printf functionality.
 *
 * @file    /_asnx_lib_/util/printf.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/13 $
 *
 * @see     https://github.com/mpaland/printf
 *****/


/***** INCLUDES *******************************************************/
#include "printf.h"
/*** STD ***/
#include <stdbool.h>
#include <stdint.h>
#include <float.h>
/*** ASNX LIB ***/
#include "uart/uart.h"


/***** STRUCTURES *****************************************************/
/* Output function type */
typedef void (*out_fct_type)(char character, void* buffer, size_t idx, size_t maxlen);

/* Wrapper (used as buffer) for output function type */
typedef struct {
    void  (*fct)(char character, void* arg);
    void* arg;
} out_fct_wrap_type;


/***** GLOBAL VARIABLES ***********************************************/
/* UART structures for function callbacks */
void (*_printf_putc)(char c) = NULL;


/***** LOCAL FUNCTION PROTOTYPES **********************************************/
// forward declaration so that _ftoa can switch to exp notation for values > PRINTF_MAX_FLOAT
static size_t _etoa(out_fct_type out, char* buffer, size_t idx, size_t maxlen, double value, unsigned int prec, unsigned int width, unsigned int flags);


/***** WRAPPER FUNCTIONS ******************************************************/
/* Internal buffer output */
static inline void _out_buffer(char character, void* buffer, size_t idx, size_t maxlen) {
    if (idx < maxlen) {
        ((char*)buffer)[idx] = character;
    }
}

/* Internal null output */
static inline void _out_null(char character, void* buffer, size_t idx, size_t maxlen) {
    (void)character; (void)buffer; (void)idx; (void)maxlen;
}

/* Internal _out_char wrapper */
static inline void _out_char(char character, void* buffer, size_t idx, size_t maxlen) {
    (void)buffer; (void)idx; (void)maxlen;
    if (character) {
        /* Check if putc callback function is already set */
        if (_printf_putc != NULL) {
            // Call putc function */
            _printf_putc(character);
        }
    }
}

/* Internal output function wrapper */
static inline void _out_fct(char character, void* buffer, size_t idx, size_t maxlen) {
    (void)idx; (void)maxlen;
    if (character) {
        // buffer is the output fct pointer
        ((out_fct_wrap_type*)buffer)->fct(character, ((out_fct_wrap_type*)buffer)->arg);
    }
}

/* Internal secure strlen
 * \return The length of the string (excluding the terminating 0) limited by 'maxsize' */
static inline unsigned int _strnlen_s(const char* str, size_t maxsize) {
    const char* s;
    for (s = str; *s && maxsize--; ++s);
    return (unsigned int)(s - str);
}

/* Internal test if char is a digit (0-9)
 * \return true if char is a digit */
static inline bool _is_digit(char ch) {
    return (ch >= '0') && (ch <= '9');
}


/***** FUNCTIONS **************************************************************/
/***
 * Initialize the printf function (set putc function).
 ***/
void printf_init(void (*callback)(char c)) {
    /* Set the putc function */
    _printf_putc = callback;
}


// internal ASCII string to unsigned int conversion
static unsigned int _atoi(const char** str) {
    unsigned int i = 0U;
    while (_is_digit(**str)) {
        i = i * 10U + (unsigned int)(*((*str)++) - '0');
    }
    return i;
}


// output the specified string in reverse, taking care of any zero-padding
static size_t _out_rev(out_fct_type out, char* buffer, size_t idx, size_t maxlen, const char* buf, size_t len, unsigned int width, unsigned int flags) {
    size_t i;
    const size_t start_idx = idx;

    // pad spaces up to given width
    if (!(flags & PRINTF_FLAGS_LEFT) && !(flags & PRINTF_FLAGS_ZEROPAD)) {
        for (i = len; i < width; i++) {
            out(' ', buffer, idx++, maxlen);
        }
    }

    // reverse string
    while (len) {
        out(buf[--len], buffer, idx++, maxlen);
    }

    // append pad spaces up to given width
    if (flags & PRINTF_FLAGS_LEFT) {
        while (idx - start_idx < width) {
            out(' ', buffer, idx++, maxlen);
        }
    }

    return idx;
}


// internal itoa format
static size_t _ntoa_format(out_fct_type out, char* buffer, size_t idx, size_t maxlen, char* buf, size_t len, bool negative, unsigned int base, unsigned int prec, unsigned int width, unsigned int flags) {
    // pad leading zeros
    if (!(flags & PRINTF_FLAGS_LEFT)) {
        if (width && (flags & PRINTF_FLAGS_ZEROPAD) && (negative || (flags & (PRINTF_FLAGS_PLUS | PRINTF_FLAGS_SPACE)))) {
            width--;
        }
        while ((len < prec) && (len < PRINTF_NTOA_BUFFER_SIZE)) {
            buf[len++] = '0';
        }
        while ((flags & PRINTF_FLAGS_ZEROPAD) && (len < width) && (len < PRINTF_NTOA_BUFFER_SIZE)) {
            buf[len++] = '0';
        }
    }

    // handle hash
    if (flags & PRINTF_FLAGS_HASH) {
        if (!(flags & PRINTF_FLAGS_PRECISION) && len && ((len == prec) || (len == width))) {
            len--;
            if (len && (base == 16U)) {
                len--;
            }
        }
        if ((base == 16U) && !(flags & PRINTF_FLAGS_UPPERCASE) && (len < PRINTF_NTOA_BUFFER_SIZE)) {
            buf[len++] = 'x';
        } else if ((base == 16U) && (flags & PRINTF_FLAGS_UPPERCASE) && (len < PRINTF_NTOA_BUFFER_SIZE)) {
            buf[len++] = 'X';
        } else if ((base == 2U) && (len < PRINTF_NTOA_BUFFER_SIZE)) {
            buf[len++] = 'b';
        }
        if (len < PRINTF_NTOA_BUFFER_SIZE) {
            buf[len++] = '0';
        }
    }

    if (len < PRINTF_NTOA_BUFFER_SIZE) {
        if (negative) {
            buf[len++] = '-';
        } else if (flags & PRINTF_FLAGS_PLUS) {
            buf[len++] = '+';  // ignore the space if the '+' exists
        } else if (flags & PRINTF_FLAGS_SPACE) {
            buf[len++] = ' ';
        }
    }

    return _out_rev(out, buffer, idx, maxlen, buf, len, width, flags);
}


// internal itoa for 'long' type
static size_t _ntoa_long(out_fct_type out, char* buffer, size_t idx, size_t maxlen, unsigned long value, bool negative, unsigned long base, unsigned int prec, unsigned int width, unsigned int flags) {
    char buf[PRINTF_NTOA_BUFFER_SIZE];
    size_t len = 0U;

    // no hash for 0 values
    if (!value) {
        flags &= ~PRINTF_FLAGS_HASH;
    }

    // write if precision != 0 and value is != 0
    if (!(flags & PRINTF_FLAGS_PRECISION) || value) {
        do {
            const char digit = (char)(value % base);
            buf[len++] = digit < 10 ? '0' + digit : (flags & PRINTF_FLAGS_UPPERCASE ? 'A' : 'a') + digit - 10;
            value /= base;
        } while (value && (len < PRINTF_NTOA_BUFFER_SIZE));
    }

    return _ntoa_format(out, buffer, idx, maxlen, buf, len, negative, (unsigned int)base, prec, width, flags);
}


static size_t _ntoa_long_long(out_fct_type out, char* buffer, size_t idx, size_t maxlen, unsigned long long value, bool negative, unsigned long long base, unsigned int prec, unsigned int width, unsigned int flags) {
    char buf[PRINTF_NTOA_BUFFER_SIZE];
    size_t len = 0U;

    // no hash for 0 values
    if (!value) {
        flags &= ~PRINTF_FLAGS_HASH;
    }

    // write if precision != 0 and value is != 0
    if (!(flags & PRINTF_FLAGS_PRECISION) || value) {
        do {
            const char digit = (char)(value % base);
            buf[len++] = digit < 10 ? '0' + digit : (flags & PRINTF_FLAGS_UPPERCASE ? 'A' : 'a') + digit - 10;
            value /= base;
        } while (value && (len < PRINTF_NTOA_BUFFER_SIZE));
    }

    return _ntoa_format(out, buffer, idx, maxlen, buf, len, negative, (unsigned int)base, prec, width, flags);
}


// internal ftoa for fixed decimal floating point
static size_t _ftoa(out_fct_type out, char* buffer, size_t idx, size_t maxlen, double value, unsigned int prec, unsigned int width, unsigned int flags) {
    char buf[PRINTF_FTOA_BUFFER_SIZE];
    size_t len  = 0U;
    double diff = 0.0;

    // powers of 10
    static const double pow10[] = { 1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000 };

    // test for special values
    if (value != value) {
        return _out_rev(out, buffer, idx, maxlen, "nan", 3, width, flags);
    }
    if (value < -DBL_MAX) {
        return _out_rev(out, buffer, idx, maxlen, "fni-", 4, width, flags);
    }
    if (value > DBL_MAX) {
        return _out_rev(out, buffer, idx, maxlen, (flags & PRINTF_FLAGS_PLUS) ? "fni+" : "fni", (flags & PRINTF_FLAGS_PLUS) ? 4U : 3U, width, flags);
    }

    // test for very large values
    // standard printf behavior is to print EVERY whole number digit -- which could be 100s of characters overflowing your buffers == bad
    if ((value > PRINTF_MAX_FLOAT) || (value < -PRINTF_MAX_FLOAT)) {
        return _etoa(out, buffer, idx, maxlen, value, prec, width, flags);
    }

    // test for negative
    bool negative = false;
    if (value < 0) {
        negative = true;
        value = 0 - value;
    }

    // set default precision, if not set explicitly
    if (!(flags & PRINTF_FLAGS_PRECISION)) {
        prec = PRINTF_DEFAULT_FLOAT_PRECISION;
    }
    // limit precision to 9, cause a prec >= 10 can lead to overflow errors
    while ((len < PRINTF_FTOA_BUFFER_SIZE) && (prec > 9U)) {
        buf[len++] = '0';
        prec--;
    }

    int whole = (int)value;
    double tmp = (value - whole) * pow10[prec];
    unsigned long frac = (unsigned long)tmp;
    diff = tmp - frac;

    if (diff > 0.5) {
        ++frac;
        // handle rollover, e.g. case 0.99 with prec 1 is 1.0
        if (frac >= pow10[prec]) {
            frac = 0;
            ++whole;
        }
    } else if (diff < 0.5) {
        
    } else if ((frac == 0U) || (frac & 1U)) {
        // if halfway, round up if odd OR if last digit is 0
        ++frac;
    }

    if (prec == 0U) {
        diff = value - (double)whole;
        if ((!(diff < 0.5) || (diff > 0.5)) && (whole & 1)) {
            // exactly 0.5 and ODD, then round up
            // 1.5 -> 2, but 2.5 -> 2
            ++whole;
        }
    } else {
        unsigned int count = prec;
        // now do fractional part, as an unsigned number
        while (len < PRINTF_FTOA_BUFFER_SIZE) {
          --count;
          buf[len++] = (char)(48U + (frac % 10U));
          if (!(frac /= 10U)) {
            break;
          }
        }
        // add extra 0s
        while ((len < PRINTF_FTOA_BUFFER_SIZE) && (count-- > 0U)) {
            buf[len++] = '0';
        }
        if (len < PRINTF_FTOA_BUFFER_SIZE) {
            // add decimal
            buf[len++] = '.';
        }
    }

    // do whole part, number is reversed
    while (len < PRINTF_FTOA_BUFFER_SIZE) {
        buf[len++] = (char)(48 + (whole % 10));
        if (!(whole /= 10)) {
            break;
        }
    }

    // pad leading zeros
    if (!(flags & PRINTF_FLAGS_LEFT) && (flags & PRINTF_FLAGS_ZEROPAD)) {
        if (width && (negative || (flags & (PRINTF_FLAGS_PLUS | PRINTF_FLAGS_SPACE)))) {
            width--;
        }
        while ((len < width) && (len < PRINTF_FTOA_BUFFER_SIZE)) {
            buf[len++] = '0';
        }
    }

    if (len < PRINTF_FTOA_BUFFER_SIZE) {
        if (negative) {
            buf[len++] = '-';
        } else if (flags & PRINTF_FLAGS_PLUS) {
            buf[len++] = '+';  // ignore the space if the '+' exists
        } else if (flags & PRINTF_FLAGS_SPACE) {
            buf[len++] = ' ';
        }
    }

    return _out_rev(out, buffer, idx, maxlen, buf, len, width, flags);
}


// internal ftoa variant for exponential floating-point type, contributed by Martijn Jasperse <m.jasperse@gmail.com>
static size_t _etoa(out_fct_type out, char* buffer, size_t idx, size_t maxlen, double value, unsigned int prec, unsigned int width, unsigned int flags) {
    // check for NaN and special values
    if ((value != value) || (value > DBL_MAX) || (value < -DBL_MAX)) {
        return _ftoa(out, buffer, idx, maxlen, value, prec, width, flags);
    }

    // determine the sign
    const bool negative = value < 0;
    if (negative) {
        value = -value;
    }

    // default precision
    if (!(flags & PRINTF_FLAGS_PRECISION)) {
        prec = PRINTF_DEFAULT_FLOAT_PRECISION;
    }

    // determine the decimal exponent
    // based on the algorithm by David Gay (https://www.ampl.com/netlib/fp/dtoa.c)
    union {
        uint64_t U;
        double   F;
    } conv;

    conv.F = value;
    int exp2 = (int)((conv.U >> 52U) & 0x07FFU) - 1023;           // effectively log2
    conv.U = (conv.U & ((1ULL << 52U) - 1U)) | (1023ULL << 52U);  // drop the exponent so conv.F is now in [1,2)
    // now approximate log10 from the log2 integer part and an expansion of ln around 1.5
    int expval = (int)(0.1760912590558 + exp2 * 0.301029995663981 + (conv.F - 1.5) * 0.289529654602168);
    // now we want to compute 10^expval but we want to be sure it won't overflow
    exp2 = (int)(expval * 3.321928094887362 + 0.5);
    const double z  = expval * 2.302585092994046 - exp2 * 0.6931471805599453;
    const double z2 = z * z;
    conv.U = (uint64_t)(exp2 + 1023) << 52U;
    // compute exp(z) using continued fractions, see https://en.wikipedia.org/wiki/Exponential_function#Continued_fractions_for_ex
    conv.F *= 1 + 2 * z / (2 - z + (z2 / (6 + (z2 / (10 + z2 / 14)))));
    // correct for rounding errors
    if (value < conv.F) {
        expval--;
        conv.F /= 10;
    }

    // the exponent format is "%+03d" and largest value is "307", so set aside 4-5 characters
    unsigned int minwidth = ((expval < 100) && (expval > -100)) ? 4U : 5U;

    // in "%g" mode, "prec" is the number of *significant figures* not decimals
    if (flags & PRINTF_FLAGS_ADAPT_EXP) {
        // do we want to fall-back to "%f" mode?
        if ((value >= 1e-4) && (value < 1e6)) {
            if ((int)prec > expval) {
                prec = (unsigned)((int)prec - expval - 1);
            } else {
                prec = 0;
            }
            flags |= PRINTF_FLAGS_PRECISION;   // make sure _ftoa respects precision
            // no characters in exponent
            minwidth = 0U;
            expval   = 0;
        } else {
            // we use one sigfig for the whole part
            if ((prec > 0) && (flags & PRINTF_FLAGS_PRECISION)) {
                --prec;
            }
        }
    }

    // will everything fit?
    unsigned int fwidth = width;
    if (width > minwidth) {
        // we didn't fall-back so subtract the characters required for the exponent
        fwidth -= minwidth;
    } else {
        // not enough characters, so go back to default sizing
        fwidth = 0U;
    }
    if ((flags & PRINTF_FLAGS_LEFT) && minwidth) {
        // if we're padding on the right, DON'T pad the floating part
        fwidth = 0U;
    }

    // rescale the float value
    if (expval) {
        value /= conv.F;
    }

    // output the floating part
    const size_t start_idx = idx;
    idx = _ftoa(out, buffer, idx, maxlen, negative ? -value : value, prec, fwidth, flags & ~PRINTF_FLAGS_ADAPT_EXP);

    // output the exponent part
    if (minwidth) {
        // output the exponential symbol
        out((flags & PRINTF_FLAGS_UPPERCASE) ? 'E' : 'e', buffer, idx++, maxlen);
        // output the exponent value
        idx = _ntoa_long(out, buffer, idx, maxlen, (expval < 0) ? -expval : expval, expval < 0, 10, 0, minwidth-1, PRINTF_FLAGS_ZEROPAD | PRINTF_FLAGS_PLUS);
        // might need to right-pad spaces
        if (flags & PRINTF_FLAGS_LEFT) {
            while (idx - start_idx < width) {
                out(' ', buffer, idx++, maxlen);
            }
        }
    }
    return idx;
}


// internal vsnprintf
static int _vsnprintf(out_fct_type out, char* buffer, const size_t maxlen, const char* format, va_list va) {
    unsigned int flags, width, precision, n;
    size_t idx = 0U;

    if (!buffer) {
        // use null output function
        out = _out_null;
    }

    while (*format) {
        // format specifier?  %[flags][width][.precision][length]
        if (*format != '%') {
            // no
            out(*format, buffer, idx++, maxlen);
            format++;
            continue;
        } else {
            // yes, evaluate it
            format++;
        }

        // evaluate flags
        flags = 0U;
        do {
            switch (*format) {
                case '0': flags |= PRINTF_FLAGS_ZEROPAD; format++; n = 1U; break;
                case '-': flags |= PRINTF_FLAGS_LEFT;    format++; n = 1U; break;
                case '+': flags |= PRINTF_FLAGS_PLUS;    format++; n = 1U; break;
                case ' ': flags |= PRINTF_FLAGS_SPACE;   format++; n = 1U; break;
                case '#': flags |= PRINTF_FLAGS_HASH;    format++; n = 1U; break;
                default :                                   n = 0U; break;
            }
        } while (n);

        // evaluate width field
        width = 0U;
        if (_is_digit(*format)) {
            width = _atoi(&format);
        } else if (*format == '*') {
            const int w = va_arg(va, int);
            if (w < 0) {
                flags |= PRINTF_FLAGS_LEFT;    // reverse padding
                width = (unsigned int)-w;
            } else {
                width = (unsigned int)w;
            }
            format++;
        }

        // evaluate precision field
        precision = 0U;
        if (*format == '.') {
            flags |= PRINTF_FLAGS_PRECISION;
            format++;
            if (_is_digit(*format)) {
                precision = _atoi(&format);
            } else if (*format == '*') {
                const int prec = (int)va_arg(va, int);
                precision = prec > 0 ? (unsigned int)prec : 0U;
                format++;
            }
        }

        // evaluate length field
        switch (*format) {
            case 'l' :
                flags |= PRINTF_FLAGS_LONG;
                format++;
                if (*format == 'l') {
                    flags |= PRINTF_FLAGS_LONG_LONG;
                    format++;
                }
                break;
            case 'h' :
                flags |= PRINTF_FLAGS_SHORT;
                format++;
                if (*format == 'h') {
                    flags |= PRINTF_FLAGS_CHAR;
                    format++;
                }
                break;
            case 't' :
                flags |= (sizeof(ptrdiff_t) == sizeof(long) ? PRINTF_FLAGS_LONG : PRINTF_FLAGS_LONG_LONG);
                format++;
                break;
            case 'j' :
                flags |= (sizeof(intmax_t) == sizeof(long) ? PRINTF_FLAGS_LONG : PRINTF_FLAGS_LONG_LONG);
                format++;
                break;
            case 'z' :
                flags |= (sizeof(size_t) == sizeof(long) ? PRINTF_FLAGS_LONG : PRINTF_FLAGS_LONG_LONG);
                format++;
                break;
            default :
                break;
        }

        // evaluate specifier
        switch (*format) {
            case 'd' :
            case 'i' :
            case 'u' :
            case 'x' :
            case 'X' :
            case 'o' :
            case 'b' : {
                // set the base
                unsigned int base;
                if (*format == 'x' || *format == 'X') {
                    base = 16U;
                } else if (*format == 'o') {
                    base =  8U;
                } else if (*format == 'b') {
                    base =  2U;
                } else {
                    base = 10U;
                    flags &= ~PRINTF_FLAGS_HASH;   // no hash for dec format
                }
                // uppercase
                if (*format == 'X') {
                    flags |= PRINTF_FLAGS_UPPERCASE;
                }

                // no plus or space flag for u, x, X, o, b
                if ((*format != 'i') && (*format != 'd')) {
                    flags &= ~(PRINTF_FLAGS_PLUS | PRINTF_FLAGS_SPACE);
                }

                // ignore '0' flag when precision is given
                if (flags & PRINTF_FLAGS_PRECISION) {
                    flags &= ~PRINTF_FLAGS_ZEROPAD;
                }

                // convert the integer
                if ((*format == 'i') || (*format == 'd')) {
                    // signed
                    if (flags & PRINTF_FLAGS_LONG_LONG) {
                        const long long value = va_arg(va, long long);
                        idx = _ntoa_long_long(out, buffer, idx, maxlen, (unsigned long long)(value > 0 ? value : 0 - value), value < 0, base, precision, width, flags);
                    } else if (flags & PRINTF_FLAGS_LONG) {
                        const long value = va_arg(va, long);
                        idx = _ntoa_long(out, buffer, idx, maxlen, (unsigned long)(value > 0 ? value : 0 - value), value < 0, base, precision, width, flags);
                    } else {
                        const int value = (flags & PRINTF_FLAGS_CHAR) ? (char)va_arg(va, int) : (flags & PRINTF_FLAGS_SHORT) ? (short int)va_arg(va, int) : va_arg(va, int);
                        idx = _ntoa_long(out, buffer, idx, maxlen, (unsigned int)(value > 0 ? value : 0 - value), value < 0, base, precision, width, flags);
                    }
                } else {
                    // unsigned
                    if (flags & PRINTF_FLAGS_LONG_LONG) {
                        idx = _ntoa_long_long(out, buffer, idx, maxlen, va_arg(va, unsigned long long), false, base, precision, width, flags);
                    } else if (flags & PRINTF_FLAGS_LONG) {
                        idx = _ntoa_long(out, buffer, idx, maxlen, va_arg(va, unsigned long), false, base, precision, width, flags);
                    } else {
                        const unsigned int value = (flags & PRINTF_FLAGS_CHAR) ? (unsigned char)va_arg(va, unsigned int) : (flags & PRINTF_FLAGS_SHORT) ? (unsigned short int)va_arg(va, unsigned int) : va_arg(va, unsigned int);
                        idx = _ntoa_long(out, buffer, idx, maxlen, value, false, base, precision, width, flags);
                    }
                }
                format++;
                break;
            }
            case 'f' :
            case 'F' :
                if (*format == 'F') {
                    flags |= PRINTF_FLAGS_UPPERCASE;
                }
                idx = _ftoa(out, buffer, idx, maxlen, va_arg(va, double), precision, width, flags);
                format++;
                break;
            case 'e':
            case 'E':
            case 'g':
            case 'G':
                if ((*format == 'g')||(*format == 'G')) {
                    flags |= PRINTF_FLAGS_ADAPT_EXP;
                }
                if ((*format == 'E')||(*format == 'G')) {
                    flags |= PRINTF_FLAGS_UPPERCASE;
                }
                idx = _etoa(out, buffer, idx, maxlen, va_arg(va, double), precision, width, flags);
                format++;
                break;
            case 'c' : {
                unsigned int l = 1U;
                // pre padding
                if (!(flags & PRINTF_FLAGS_LEFT)) {
                    while (l++ < width) {
                        out(' ', buffer, idx++, maxlen);
                    }
                }
                // char output
                out((char)va_arg(va, int), buffer, idx++, maxlen);
                // post padding
                if (flags & PRINTF_FLAGS_LEFT) {
                    while (l++ < width) {
                        out(' ', buffer, idx++, maxlen);
                    }
                }
                format++;
                break;
            }
            case 's' : {
                const char* p = va_arg(va, char*);
                unsigned int l = _strnlen_s(p, precision ? precision : (size_t)-1);
                // pre padding
                if (flags & PRINTF_FLAGS_PRECISION) {
                    l = (l < precision ? l : precision);
                }
                if (!(flags & PRINTF_FLAGS_LEFT)) {
                    while (l++ < width) {
                        out(' ', buffer, idx++, maxlen);
                    }
                }
                // string output
                while ((*p != 0) && (!(flags & PRINTF_FLAGS_PRECISION) || precision--)) {
                    out(*(p++), buffer, idx++, maxlen);
                }
                // post padding
                if (flags & PRINTF_FLAGS_LEFT) {
                    while (l++ < width) {
                        out(' ', buffer, idx++, maxlen);
                    }
                }
                format++;
                break;
            }
            case 'p' : {
                width = sizeof(void*) * 2U;
                flags |= PRINTF_FLAGS_ZEROPAD | PRINTF_FLAGS_UPPERCASE;
                const bool is_ll = sizeof(uintptr_t) == sizeof(long long);
                if (is_ll) {
                    idx = _ntoa_long_long(out, buffer, idx, maxlen, (uintptr_t)va_arg(va, void*), false, 16U, precision, width, flags);
                } else {
                    idx = _ntoa_long(out, buffer, idx, maxlen, (unsigned long)((uintptr_t)va_arg(va, void*)), false, 16U, precision, width, flags);
                }
                format++;
                break;
            }
            case '%' :
                out('%', buffer, idx++, maxlen);
                format++;
                break;

                default :
                out(*format, buffer, idx++, maxlen);
                format++;
                break;
        }
    }

    // termination
    out((char)0, buffer, idx < maxlen ? idx : maxlen - 1U, maxlen);

    // return written chars without terminating \0
    return (int)idx;
}

///////////////////////////////////////////////////////////////////////////////
int printf_(const char* format, ...) {
    va_list va;
    va_start(va, format);
    char buffer[1];
    const int ret = _vsnprintf(_out_char, buffer, (size_t)-1, format, va);
    va_end(va);
    return ret;
}

int sprintf_(char* buffer, const char* format, ...) {
    va_list va;
    va_start(va, format);
    const int ret = _vsnprintf(_out_buffer, buffer, (size_t)-1, format, va);
    va_end(va);
    return ret;
}

int snprintf_(char* buffer, size_t count, const char* format, ...) {
    va_list va;
    va_start(va, format);
    const int ret = _vsnprintf(_out_buffer, buffer, count, format, va);
    va_end(va);
    return ret;
}

int vprintf_(const char* format, va_list va) {
    char buffer[1];
    return _vsnprintf(_out_char, buffer, (size_t)-1, format, va);
}

int vsnprintf_(char* buffer, size_t count, const char* format, va_list va) {
    return _vsnprintf(_out_buffer, buffer, count, format, va);
}

int fctprintf(void (*out)(char character, void* arg), void* arg, const char* format, ...) {
    va_list va;
    va_start(va, format);
    const out_fct_wrap_type out_fct_wrap = { out, arg };
    const int ret = _vsnprintf(_out_fct, (char*)(uintptr_t)&out_fct_wrap, (size_t)-1, format, va);
    va_end(va);
    return ret;
}
