/**
 *  Header file for AVR UART functionality.
 */

#ifndef _AVR_UART_H_
#define _AVR_UART_H_

/***** INCLUDES ***************************************************************/
#include <stdint.h>


/***** MACROS *****************************************************************/
/*** BASIC CPU FREQUENCY ***/
#ifndef F_CPU
# warning "F_CPU not defined for \"timer.h\""
# define F_CPU 16000000UL
#endif
/*** UART SPECIFIC ***/
/* Use 2X mode for higher baud rate (default) */
#define UART_USE_2X                 (0)
/* Function return values */
#define UART_RET_OK                 (0)
#define UART_RET_ERROR              (-1)


/***** GLOBAL VARIABLES *******************************************************/


/***** ENUMERATION ************************************************************/


/***** STRUCTURES *************************************************************/
typedef struct {
    /* RX callback function (receive) */
    void (*f_rx)(void);
    /* TX callback function (transmit) */
    void (*f_tx)(void);
    /* EMTPY callback function (receive) */
    void (*f_empty)(void);
} uart_isr_t;


/***** FUNCTION PROTOTYPES ****************************************************/
/*** GENERAL ***/
void uart_init(void);
void uart_set_baudrate(uint32_t baudrate);
/* ISR-specific */
void uart_interrupt_enable(void);
void uart_interrupt_disable(void);
void uart_set_callback_rx(void (*callback)());
void uart_set_callback_tx(void (*callback)());
void uart_set_callback_empty(void (*callback)());
/*** BLOCKING ***/
void uart_putc(char c);
void uart_puts(char* s);
int8_t uart_write_blocking(uint8_t* data, uint16_t len);
uint8_t uart_getc(void);
int8_t uart_gets(uint8_t* s, uint16_t len);
/*** NON-BLOCKING ***/
int8_t uart_write(uint8_t* data, uint16_t len);
uint16_t uart_read(uint8_t* data, uint16_t len);
uint16_t uart_rx_buffer_cnt(void);
uint16_t uart_tx_buffer_cnt(void);
void uart_rx_flush(void);
void uart_tx_flush(void);


/***** INLINE FUNCTIONS *******************************************************/


#endif // _AVR_UART_H_
