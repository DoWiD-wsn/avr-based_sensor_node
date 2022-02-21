/*!
 * @brief   ASN(x) UART library -- header file
 *
 * Library to support the use of the UART (blocking and non-blocking).
 *
 * @file    /_asnx_lib_/uart/uart.h
 * @author  Dominik Widhalm
 * @version 1.2.2
 * @date    2022/01/24
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
#define UART0_ENABLE_INTERRUPT      (1)
/*! Enable UART1 interrupts */
#define UART1_ENABLE_INTERRUPT      (0)


/***** INCLUDES *******************************************************/
/*** STD ***/
#include <stdint.h>
/*** AVR ***/
#include <avr/io.h>
#if UART0_ENABLE_INTERRUPT || UART1_ENABLE_INTERRUPT
#  include <avr/interrupt.h>
/*** ASNX LIB ***/
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
void uart0_init(uint32_t baudrate);
void uart1_init(uint32_t baudrate);
void uart0_enable(void);
void uart1_enable(void);
void uart0_disable(void);
void uart1_disable(void);
void uart0_set_baudrate(uint32_t baudrate);
void uart1_set_baudrate(uint32_t baudrate);
/*** POLLING ***/
void uart0_write_char(char character);
void uart1_write_char(char character);
void uart0_write_byte(uint8_t byte);
void uart1_write_byte(uint8_t byte);
int8_t uart0_write_block(uint8_t* data, uint16_t len);
int8_t uart1_write_block(uint8_t* data, uint16_t len);
uint8_t uart0_read_byte(void);
uint8_t uart1_read_byte(void);
int8_t uart0_read_block(uint8_t* data, uint16_t len);
int8_t uart1_read_block(uint8_t* data, uint16_t len);
/*** ISR-DRIVEN ***/
/* UART0 */
#if UART0_ENABLE_INTERRUPT
void uart0_interrupt_enable(void);
void uart0_interrupt_disable(void);
void uart0_set_callback_rx(void (*callback)(void));
void uart0_set_callback_tx(void (*callback)(void));
void uart0_set_callback_empty(void (*callback)(void));
int8_t uart0_put_byte(uint8_t byte);
int8_t uart0_pop_byte(uint8_t* byte);
int8_t uart0_put_block(uint8_t* data, uint16_t len);
int8_t uart0_pop_block(uint8_t* data, uint16_t len, uint8_t* cnt);
uint8_t uart0_rx_buffer_cnt(void);
uint8_t uart0_tx_buffer_cnt(void);
void uart0_rx_cb_flush(void);
void uart0_tx_cb_flush(void);
#endif
/* UART1 */
#if UART1_ENABLE_INTERRUPT
void uart1_interrupt_enable(void);
void uart1_interrupt_disable(void);
void uart1_set_callback_rx(void (*callback)(void));
void uart1_set_callback_tx(void (*callback)(void));
void uart1_set_callback_empty(void (*callback)(void));
int8_t uart1_put_byte(uint8_t byte);
int8_t uart1_pop_byte(uint8_t* byte);
int8_t uart1_put_block(uint8_t* data, uint16_t len);
int8_t uart1_pop_block(uint8_t* data, uint16_t len, uint8_t* cnt);
uint8_t uart1_rx_buffer_cnt(void);
uint8_t uart1_tx_buffer_cnt(void);
void uart1_rx_cb_flush(void);
void uart1_tx_cb_flush(void);
#endif

#endif // _ASNX_UART_H_
