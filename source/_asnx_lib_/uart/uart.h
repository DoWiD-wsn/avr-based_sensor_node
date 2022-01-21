/*!
 * @brief   ASN(x) UART library -- header file
 *
 * Library to support the use of the UART (blocking and non-blocking).
 *
 * @file    /_asnx_lib_/uart/uart.h
 * @author  Dominik Widhalm
 * @version 1.2.1
 * @date    2022/01/21
 */

#ifndef _ASNX_UART_H_
#define _ASNX_UART_H_

/***** DEFINES ********************************************************/
/*! CPU frequency (F_CPU) */
#ifndef F_CPU
# warning "F_CPU not defined for \"uart.h\""
# define F_CPU 4000000UL
#endif
/*** UART specifics ***/
/*! Use 2X mode for higher baud rate */
#define UART_USE_2X                 (0)
/*! Enable UART0 interrupts */
#define UART0_ENABLE_INTERRUPT      (0)
/*! Enable UART1 interrupts */
#define UART1_ENABLE_INTERRUPT      (0)


/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
/*** AVR ***/
#include <avr/io.h>
#if UART0_ENABLE_INTERRUPT || UART1_ENABLE_INTERRUPT
#include <avr/interrupt.h>
#endif
/*** ASNX LIB ***/
#if UART0_ENABLE_INTERRUPT || UART1_ENABLE_INTERRUPT
#  include "util/cbuffer.h"
#endif


/***** ENUMERATION ****************************************************/
/*! Enumeration for the UART function returns values */
typedef enum {
    UART_RET_OK                         = 0,    /**< SUCCESS */
    UART_RET_ERROR                      = -1,   /**< ERROR: general error */
    UART_RET_TIMEOUT                    = -2    /**< ERROR: timeout */
} UART_RET_t;


/***** STRUCTURES *****************************************************/
/*!
 * A structure to store the UART function callbacks.
 */
#if UART0_ENABLE_INTERRUPT || UART1_ENABLE_INTERRUPT
typedef struct {
    void (*f_rx)(void);     /**< RX callback function (receive) */
    void (*f_tx)(void);     /**< TX callback function (transmit) */
    void (*f_empty)(void);  /**< EMTPY callback function (receive) */
} uart_isr_t;
#endif


/***** FUNCTION PROTOTYPES ********************************************/
/*** GENERAL ***/
void uart0_init(void);
void uart1_init(void);
void uart0_enable(void);
void uart1_enable(void);
void uart0_disable(void);
void uart1_disable(void);
void uart0_set_baudrate(uint32_t baudrate);
void uart1_set_baudrate(uint32_t baudrate);
/* Check status flags */
uint8_t uart0_tx_ready(void);
uint8_t uart1_tx_ready(void);
uint8_t uart0_rx_ready(void);
uint8_t uart1_rx_ready(void);
void uart0_rx_flush(void);
void uart1_rx_flush(void);
/*** BLOCKING ***/
void uart0_putc(char c);
void uart1_putc(char c);
void uart0_puts(char* s);
void uart1_puts(char* s);
UART_RET_t uart0_write_blocking(uint8_t* data, uint16_t len);
UART_RET_t uart1_write_blocking(uint8_t* data, uint16_t len);
char uart0_getc(void);
char uart1_getc(void);
UART_RET_t uart0_gets(char* s, uint16_t len);
UART_RET_t uart1_gets(char* s, uint16_t len);
/*** NON-BLOCKING ***/
#if UART0_ENABLE_INTERRUPT
void uart0_interrupt_enable(void);
void uart0_interrupt_disable(void);
void uart0_set_callback_rx(void (*callback)(void));
void uart0_set_callback_tx(void (*callback)(void));
void uart0_set_callback_empty(void (*callback)(void));
UART_RET_t uart0_write(uint8_t* data, uint16_t len);
UART_RET_t uart0_read(uint8_t* data, uint16_t len, uint8_t* cnt);
uint8_t uart0_rx_buffer_cnt(void);
uint8_t uart0_tx_buffer_cnt(void);
void uart0_rx_cb_flush(void);
void uart0_tx_cb_flush(void);
#endif
#if UART1_ENABLE_INTERRUPT
void uart1_interrupt_enable(void);
void uart1_interrupt_disable(void);
void uart1_set_callback_rx(void (*callback)(void));
void uart1_set_callback_tx(void (*callback)(void));
void uart1_set_callback_empty(void (*callback)(void));
UART_RET_t uart1_write(uint8_t* data, uint16_t len);
UART_RET_t uart1_read(uint8_t* data, uint16_t len, uint8_t* cnt);
uint8_t uart1_rx_buffer_cnt(void);
uint8_t uart1_tx_buffer_cnt(void);
void uart1_rx_cb_flush(void);
void uart1_tx_cb_flush(void);
#endif

#endif // _ASNX_UART_H_
