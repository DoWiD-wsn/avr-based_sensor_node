/*****
 * @brief   ASN(x) UART library
 *
 * Library to support the use of the UART (blocking and non-blocking).
 *
 * @file    /_asnx_lib_/uart/uart.c
 * @author  $Author: Dominik Widhalm $
 * @version $Revision: 1.0 $
 * @date    $Date: 2021/04/13 $
 *****/

/***** INCLUDES *******************************************************/
#include "uart.h"
/*** STD ***/
#include <stddef.h>
/*** AVR ***/
#include <avr/io.h>
#include <avr/interrupt.h>
/*** ASNX LIB ***/
#include "util/cbuffer.h"


/***** DEFINES ********************************************************/
/* Calculate the BAUD register values */
#define UART_UBRR_ASYNC_NORMAL(br)  ((F_CPU / (br * 16.0)) - 1.0)
#define UART_UBRR_ASYNC_DOUBLE(br)  ((F_CPU / (br * 8.0)) - 1.0)


/***** GLOBAL VARIABLES ***********************************************/
/* Circular buffers (need to be global due to ISR) */
cbuf_t uart0_cb_rx;
cbuf_t uart0_cb_tx;
cbuf_t uart1_cb_rx;
cbuf_t uart1_cb_tx;
/* UART structures for function callbacks */
uart_isr_t uart0_isr;
uart_isr_t uart1_isr;


/***** FUNCTIONS ******************************************************/
/***
 * Initialize the UART0 interface (default: 9600 baud).
 ***/
void uart0_init(void) {
    /* Set baudrate per default to 9600 */
    uart0_set_baudrate(9600);
    
#if UART_USE_2X
    /* Enable U2X mode */
    UCSR0A |= _BV(U2X0);
#else
    /* Disable U2X mode */
    UCSR0A &= ~_BV(U2X0);
#endif
    /* Set the framing (8N1) */
    UCSR0C |= _BV(UCSZ00) | _BV(UCSZ01);
    /* Enable RX and TX */
    UCSR0B |= _BV(RXEN0) | _BV(TXEN0);
    
    /* Initialize rx/tx buffer */
    cbuf_init(&uart0_cb_rx);
    cbuf_init(&uart0_cb_tx);
}


/***
 * Initialize the UART1 interface (default: 9600 baud).
 ***/
void uart1_init(void) {
    /* Set baudrate per default to 9600 */
    uart1_set_baudrate(9600);
    
#if UART_USE_2X
    /* Enable U2X mode */
    UCSR1A |= _BV(U2X1);
#else
    /* Disable U2X mode */
    UCSR1A &= ~_BV(U2X1);
#endif
    /* Set the framing (8N1) */
    UCSR1C |= _BV(UCSZ10) | _BV(UCSZ11);
    /* Enable RX and TX */
    UCSR1B |= _BV(RXEN1) | _BV(TXEN1);
    
    /* Initialize rx/tx buffer */
    cbuf_init(&uart1_cb_rx);
    cbuf_init(&uart1_cb_tx);
}


/***
 * Set the BAUD rate of UART0.
 *
 * @param[in]   baudrate    The baud rate to be set.
 ***/
void uart0_set_baudrate(uint32_t baudrate) {
    /* Set the baudrate */
#if UART_USE_2X
    UBRR0H = (uint8_t)((uint16_t)(UART_UBRR_ASYNC_DOUBLE(baudrate)) >> 8);
    UBRR0L = (uint8_t)((uint16_t)(UART_UBRR_ASYNC_DOUBLE(baudrate)) & 0x00FF);
#else
    UBRR0H = (uint8_t)((uint16_t)(UART_UBRR_ASYNC_NORMAL(baudrate)) >> 8);
    UBRR0L = (uint8_t)((uint16_t)(UART_UBRR_ASYNC_NORMAL(baudrate)) & 0x00FF);
#endif
}


/***
 * Set the BAUD rate of UART1.
 *
 * @param[in]   baudrate    The baud rate to be set.
 ***/
void uart1_set_baudrate(uint32_t baudrate) {
    /* Set the baudrate */
#if UART_USE_2X
    UBRR1H = (uint8_t)((uint16_t)(UART_UBRR_ASYNC_DOUBLE(baudrate)) >> 8);
    UBRR1L = (uint8_t)((uint16_t)(UART_UBRR_ASYNC_DOUBLE(baudrate)) & 0x00FF);
#else
    UBRR1H = (uint8_t)((uint16_t)(UART_UBRR_ASYNC_NORMAL(baudrate)) >> 8);
    UBRR1L = (uint8_t)((uint16_t)(UART_UBRR_ASYNC_NORMAL(baudrate)) & 0x00FF);
#endif
}


/***
 * Enable the UART0 interface.
 ***/
void uart0_interrupt_enable(void) {
    /* Enable "RX Complete Interrupt" */
    UCSR0B |= _BV(RXCIE0);
}


/***
 * Enable the UART1 interface.
 ***/
void uart1_interrupt_enable(void) {
    /* Enable "RX Complete Interrupt" */
    UCSR1B |= _BV(RXCIE1);
}


/***
 * Disable the UART0 interface.
 ***/
void uart0_interrupt_disable(void) {
    /* Disable "RX Complete Interrupt" */
    UCSR0B &= ~_BV(RXCIE0);
    /* Disable "TX Complete Interrupt" */
    UCSR0B &= ~_BV(TXCIE0);
    /* Disable "USART Data Register Empty Interrupt" */
    UCSR0B &= ~_BV(UDRIE0);
}


/***
 * Disable the UART1 interface.
 ***/
void uart1_interrupt_disable(void) {
    /* Disable "RX Complete Interrupt" */
    UCSR1B &= ~_BV(RXCIE1);
    /* Disable "TX Complete Interrupt" */
    UCSR1B &= ~_BV(TXCIE1);
    /* Disable "USART Data Register Empty Interrupt" */
    UCSR1B &= ~_BV(UDRIE1);
}


/***
 * Set a UART0 RX callback function.
 *
 * @param[in]   callback    Callback function pointer
 ***/
void uart0_set_callback_rx(void (*callback)()) {
    /* Set the callback function */
    uart0_isr.f_rx = callback;
}


/***
 * Set a UART1 RX callback function.
 *
 * @param[in]   callback    Callback function pointer
 ***/
void uart1_set_callback_rx(void (*callback)()) {
    /* Set the callback function */
    uart1_isr.f_rx = callback;
}


/***
 * Set a UART0 TX callback function.
 *
 * @param[in]   callback    Callback function pointer
 ***/
void uart0_set_callback_tx(void (*callback)()) {
    /* Set the callback function */
    uart0_isr.f_tx = callback;
}


/***
 * Set a UART1 TX callback function.
 *
 * @param[in]   callback    Callback function pointer
 ***/
void uart1_set_callback_tx(void (*callback)()) {
    /* Set the callback function */
    uart1_isr.f_tx = callback;
}


/***
 * Set a UART0 EMPTY callback function.
 *
 * @param[in]   callback    Callback function pointer
 ***/
void uart0_set_callback_empty(void (*callback)()) {
    /* Set the callback function */
    uart0_isr.f_empty = callback;
}


/***
 * Set a UART1 EMPTY callback function.
 *
 * @param[in]   callback    Callback function pointer
 ***/
void uart1_set_callback_empty(void (*callback)()) {
    /* Set the callback function */
    uart1_isr.f_empty = callback;
}


/***
 * Transmit a character via UART0 (blocking).
 *
 * @param[in]   c           Character to be transmitted
 ***/
void uart0_putc(char c) {
    /* Wait for transmit buffer to be empty */
    while(!(UCSR0A & _BV(UDRE0)));
    /* Write byte to the output buffer */
    UDR0 = c;
}


/***
 * Transmit a character via UART1 (blocking).
 *
 * @param[in]   c           Character to be transmitted
 ***/
void uart1_putc(char c) {
    /* Wait for transmit buffer to be empty */
    while(!(UCSR1A & _BV(UDRE1)));
    /* Write byte to the output buffer */
    UDR1 = c;
}


/***
 * Write a string via UART0 (blocking).
 *
 * @param[in]   s           String to be transmitted
 ***/
void uart0_puts(char* s) {
    /* Transmit the string character by character */
    while(*s) {
        uart0_putc(*s++);
    }
}


/***
 * Write a string via UART1 (blocking).
 *
 * @param[in]   s           String to be transmitted
 ***/
void uart1_puts(char* s) {
    /* Transmit the string character by character */
    while(*s) {
        uart1_putc(*s++);
    }
}


/***
 * Transmit a number of bytes via UART0 (blocking).
 *
 * @param[in]   data        Pointer to the data array
 * @param[in]   len         Number of bytes to be transmitted
 * @return      OK in case of successful
 ***/
int8_t uart0_write_blocking(uint8_t* data, uint16_t len) {
    uint16_t i;
    /* Put the specified number of bytes in the TX buffer */
    for(i=0; i<len; i++) {
        /* Write the byte via UART */
        uart0_putc(data[i]);
    }
    return UART_RET_OK;
}


/***
 * Transmit a number of bytes via UART1 (blocking).
 *
 * @param[in]   data        Pointer to the data array
 * @param[in]   len         Number of bytes to be transmitted
 * @return      OK in case of successful
 ***/
int8_t uart1_write_blocking(uint8_t* data, uint16_t len) {
    uint16_t i;
    /* Put the specified number of bytes in the TX buffer */
    for(i=0; i<len; i++) {
        /* Write the byte via UART */
        uart1_putc(data[i]);
    }
    return UART_RET_OK;
}


/***
 * Receive a character from UART0 (blocking).
 *
 * @return      Character read
 ***/
uint8_t uart0_getc(void) {
    /* Wait until reception is finished */
    while(!(UCSR0A & _BV(RXC0)));
    /* Return the received byte */
    return UDR0;
}


/***
 * Receive a character from UART1 (blocking).
 *
 * @return      Character read
 ***/
uint8_t uart1_getc(void) {
    /* Wait until reception is finished */
    while(!(UCSR1A & _BV(RXC1)));
    /* Return the received byte */
    return UDR1;
}


/***
 * Receive a string from UART0 (blocking) up to a certain length.
 *
 * @param[out]  s           String to be received.
 * @param[in]   len         Maximum number of bytes to be received
 * @return      OK in case of successful
 ***/
int8_t uart0_gets(uint8_t* s, uint16_t len) {
    uint16_t index = 0;
    uint8_t temp;
    /* Read until string is finished or len is reached */
    do {
        /* Read next character */
        temp = uart0_getc();
        /* Store character in string buffer */
        s[index++] = temp;
    } while((index < len) && (temp != '\n'));
    /* Make sure the string is null terminated */
    s[index] = '\0';
    /* Return success */
    return UART_RET_OK;
}


/***
 * Receive a string from UART1 (blocking) up to a certain length.
 *
 * @param[out]  s           String to be received.
 * @param[in]   len         Maximum number of bytes to be received
 * @return      OK in case of successful
 ***/
int8_t uart1_gets(uint8_t* s, uint16_t len) {
    uint16_t index = 0;
    uint8_t temp;
    /* Read until string is finished or len is reached */
    do {
        /* Read next character */
        temp = uart1_getc();
        /* Store character in string buffer */
        s[index++] = temp;
    } while((index < len) && (temp != '\n'));
    /* Make sure the string is null terminated */
    s[index] = '\0';
    /* Return success */
    return UART_RET_OK;
}


/***
 * Write data to the UART0 TX buffer (non-blocking ISR mode).
 *
 * @param[in]   data        Pointer to the data array to be written
 * @param[in]   len         Number of bytes to be written
 * @return      OK in case of successful
 ***/
int8_t uart0_write(uint8_t* data, uint16_t len) {
    uint16_t i;
    /* Put the specified number of bytes in the TX buffer */
    for(i=0; i<len; i++) {
        /* Try to push byte on the CB */
        if(cbuf_push(&uart0_cb_tx,data[i]) != CBUF_RET_OK) {
            /* Push didn't work ...*/
            return UART_RET_ERROR;
        } else {
            /* Activate buffer empty interrupt */
            UCSR0B |= _BV(UDRIE0);
        }
    }
    /* Return success */
    return UART_RET_OK;
}


/***
 * Write data to the UART1 TX buffer (non-blocking ISR mode).
 *
 * @param[in]   data        Pointer to the data array to be written
 * @param[in]   len         Number of bytes to be written
 * @return      OK in case of successful
 ***/
int8_t uart1_write(uint8_t* data, uint16_t len) {
    uint16_t i;
    /* Put the specified number of bytes in the TX buffer */
    for(i=0; i<len; i++) {
        /* Try to push byte on the CB */
        if(cbuf_push(&uart1_cb_tx,data[i]) != CBUF_RET_OK) {
            /* Push didn't work ...*/
            return UART_RET_ERROR;
        } else {
            /* Activate buffer empty interrupt */
            UCSR1B |= _BV(UDRIE1);
        }
    }
    /* Return success */
    return UART_RET_OK;
}


/***
 * Read data from the UART0 RX buffer (non-blocking ISR mode).
 *
 * @param[out]  data        Pointer to the data array to be read
 * @param[in]   len         Maximum number of bytes to be read
 * @return      OK in case of successful
 ***/
uint16_t uart0_read(uint8_t* data, uint16_t len) {
    uint16_t i=0;
    /* Get bytes from the RX buffer */
    for(i=0; i<len; i++) {
        uint8_t tmp;
        /* Try to pop one byte from the CB */
        int8_t ret = cbuf_pop(&uart0_cb_rx, &tmp);
        /* Check if pop was successful */
        if(ret == CBUF_RET_OK) {
            /* Copy byte to data array */
            data[i] = tmp;
        } else {
            /* Nothing left to pop */
            break;
        }
    }
    /* Return success */
    return i;
}


/***
 * Read data from the UART1 RX buffer (non-blocking ISR mode).
 *
 * @param[out]  data        Pointer to the data array to be read
 * @param[in]   len         Maximum number of bytes to be read
 * @return      OK in case of successful
 ***/
uint16_t uart1_read(uint8_t* data, uint16_t len) {
    uint16_t i=0;
    /* Get bytes from the RX buffer */
    for(i=0; i<len; i++) {
        uint8_t tmp;
        /* Try to pop one byte from the CB */
        int8_t ret = cbuf_pop(&uart1_cb_rx, &tmp);
        /* Check if pop was successful */
        if(ret == CBUF_RET_OK) {
            /* Copy byte to data array */
            data[i] = tmp;
        } else {
            /* Nothing left to pop */
            break;
        }
    }
    /* Return success */
    return i;
}


/***
 * Get the amount of byte currently in the UART0 RX buffer.
 *
 * @return      Number of bytes in the UART0 RX buffer
 ***/
uint16_t uart0_rx_buffer_cnt(void) {
    /* Return number of bytes in RX buffer */
    return uart0_cb_rx.cnt;
}


/***
 * Get the amount of byte currently in the UART1 RX buffer.
 *
 * @return      Number of bytes in the UART1 RX buffer
 ***/
uint16_t uart1_rx_buffer_cnt(void) {
    /* Return number of bytes in RX buffer */
    return uart1_cb_rx.cnt;
}


/***
 * Get the amount of byte currently in the UART0 TX buffer.
 *
 * @return      Number of bytes in the UART0 TX buffer
 ***/
uint16_t uart0_tx_buffer_cnt(void) {
    /* Return number of bytes in TX buffer */
    return uart0_cb_tx.cnt;
}


/***
 * Get the amount of byte currently in the UART1 TX buffer.
 *
 * @return      Number of bytes in the UART1 TX buffer
 ***/
uint16_t uart1_tx_buffer_cnt(void) {
    /* Return number of bytes in TX buffer */
    return uart1_cb_tx.cnt;
}


/***
 * Flush the UART0 RX buffer
 ***/
void uart0_rx_flush(void) {
    /* Flush the CB */
    cbuf_flush(&uart0_cb_rx);
}


/***
 * Flush the UART1 RX buffer
 ***/
void uart1_rx_flush(void) {
    /* Flush the CB */
    cbuf_flush(&uart1_cb_rx);
}


/***
 * Flush the UART0 TX buffer
 ***/
void uart0_tx_flush(void) {
    /* Flush the CB */
    cbuf_flush(&uart0_cb_tx);
}


/***
 * Flush the UART1 TX buffer
 ***/
void uart1_tx_flush(void) {
    /* Flush the CB */
    cbuf_flush(&uart1_cb_tx);
}


/***
 * UART0 RX complete interrupt
 ***/
#if defined(__DOXYGEN__)
void USART0_RX_vect(void) {
#else
ISR(USART0_RX_vect) {
#endif
    /* Push received byte in RX buffer */
    cbuf_push(&uart0_cb_rx,UDR0);
    /* Check if a callback function was defined */
    if(uart0_isr.f_rx != NULL) {
        /* Call function */
        uart0_isr.f_rx();
    }
}


/***
 * UART1 RX complete interrupt
 ***/
#if defined(__DOXYGEN__)
void USART1_RX_vect(void) {
#else
ISR(USART1_RX_vect) {
#endif
    /* Push received byte in RX buffer */
    cbuf_push(&uart1_cb_rx,UDR1);
    /* Check if a callback function was defined */
    if(uart1_isr.f_rx != NULL) {
        /* Call function */
        uart1_isr.f_rx();
    }
}


/***
 * UART0 TX complete interrupt
 ***/
#if defined(__DOXYGEN__)
void USART0_TX_vect(void) {
#else
ISR(USART0_TX_vect) {
#endif
    /* Check if a callback function was defined */
    if(uart0_isr.f_tx != NULL) {
        /* Call function */
        uart0_isr.f_tx();
    }
}


/***
 * UART0 TX complete interrupt
 ***/
#if defined(__DOXYGEN__)
void USART1_TX_vect(void) {
#else
ISR(USART1_TX_vect) {
#endif
    /* Check if a callback function was defined */
    if(uart1_isr.f_tx != NULL) {
        /* Call function */
        uart1_isr.f_tx();
    }
}


/***
 * UART0 data register empty interrupt
 ***/
#if defined(__DOXYGEN__)
void USART0_UDRE_vect(void) {
#else
ISR(USART0_UDRE_vect) {
#endif
    uint8_t tmp;
    int8_t ret;
    /* Try to pop one byte from the CB */
    ret = cbuf_pop(&uart0_cb_tx, &tmp);
    /* Check if pop was successful */
    if(ret == CBUF_RET_OK) {
        /* Send byte via UART */
        UDR0 = tmp;
    } else {
        /* Clear receive interrupt */
        UCSR0B &= ~(_BV(UDRIE0));
        /* Check if a callback function was defined */
        if(uart0_isr.f_empty != NULL) {
            /* Call function */
            uart0_isr.f_empty();
        }
    }
}


/***
 * UART1 data register empty interrupt
 ***/
#if defined(__DOXYGEN__)
void USART0_UDRE_vect(void) {
#else
ISR(USART1_UDRE_vect) {
#endif
    uint8_t tmp;
    int8_t ret;
    /* Try to pop one byte from the CB */
    ret = cbuf_pop(&uart1_cb_tx, &tmp);
    /* Check if pop was successful */
    if(ret == CBUF_RET_OK) {
        /* Send byte via UART */
        UDR1 = tmp;
    } else {
        /* Clear receive interrupt */
        UCSR1B &= ~(_BV(UDRIE1));
        /* Check if a callback function was defined */
        if(uart1_isr.f_empty != NULL) {
            /* Call function */
            uart1_isr.f_empty();
        }
    }
}
