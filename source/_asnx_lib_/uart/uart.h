/*****
 * @brief   ASN(x) UART library
 *
 * Library to support the use of the UART (blocking and non-blocking).
 *
 * @file    /_asnx_lib_/uart/uart.h
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/13 $
 *****/

#ifndef _ASNX_UART_H_
#define _ASNX_UART_H_

/***** INCLUDES *******************************************************/
#include <stdint.h>


/***** DEFINES ********************************************************/
/*** CPU frequency (F_CPU) ***/
#ifndef F_CPU
# warning "F_CPU not defined for \"uart.h\""
# define F_CPU 4000000UL
#endif
/*** UART specifics ***/
/* Use 2X mode for higher baud rate */
#define UART_USE_2X                 (0)
/* Function return values */
#define UART_RET_OK                 (0)
#define UART_RET_ERROR              (-1)


/***** STRUCTURES *****************************************************/
/***
 * A structure to store the UART function callbacks.
 ***/
typedef struct {
    void (*f_rx)(void);     /**< RX callback function (receive) */
    void (*f_tx)(void);     /**< TX callback function (transmit) */
    void (*f_empty)(void);  /**< EMTPY callback function (receive) */
} uart_isr_t;


/***** FUNCTION PROTOTYPES ********************************************/
/*** GENERAL ***/
void uart0_init(void);
void uart1_init(void);
void uart0_set_baudrate(uint32_t baudrate);
void uart1_set_baudrate(uint32_t baudrate);
/* ISR-specific */
void uart0_interrupt_enable(void);
void uart1_interrupt_enable(void);
void uart0_interrupt_disable(void);
void uart1_interrupt_disable(void);
void uart0_set_callback_rx(void (*callback)());
void uart1_set_callback_rx(void (*callback)());
void uart0_set_callback_tx(void (*callback)());
void uart1_set_callback_tx(void (*callback)());
void uart0_set_callback_empty(void (*callback)());
void uart1_set_callback_empty(void (*callback)());
/*** BLOCKING ***/
void uart0_putc(char c);
void uart1_putc(char c);
void uart0_puts(char* s);
void uart1_puts(char* s);
int8_t uart0_write_blocking(uint8_t* data, uint16_t len);
int8_t uart1_write_blocking(uint8_t* data, uint16_t len);
uint8_t uart0_getc(void);
uint8_t uart1_getc(void);
int8_t uart0_gets(uint8_t* s, uint16_t len);
int8_t uart1_gets(uint8_t* s, uint16_t len);
/*** NON-BLOCKING ***/
int8_t uart0_write(uint8_t* data, uint16_t len);
int8_t uart1_write(uint8_t* data, uint16_t len);
uint16_t uart0_read(uint8_t* data, uint16_t len);
uint16_t uart1_read(uint8_t* data, uint16_t len);
uint16_t uart0_rx_buffer_cnt(void);
uint16_t uart1_rx_buffer_cnt(void);
uint16_t uart0_tx_buffer_cnt(void);
uint16_t uart1_tx_buffer_cnt(void);
void uart0_rx_flush(void);
void uart1_rx_flush(void);
void uart0_tx_flush(void);
void uart1_tx_flush(void);


#endif // _ASNX_UART_H_
