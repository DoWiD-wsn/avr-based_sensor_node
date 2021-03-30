/**
 *  Source file for AVR UART functionality.
 */

/***** INCLUDES ***************************************************************/
/* STD */
#include <stddef.h>
/* AVR */
#include <avr/io.h>
#include <avr/interrupt.h>
/* OWN */
#include "uart.h"
#include "util/cbuffer.h"
#include "util/fuses.h"


/***** MACROS *****************************************************************/
/*** UART CONFIG ***/
/* Macro to get the BAUD register value */
#define UART_UBRR_ASYNC_NORMAL(cpu,br)  ((cpu / (br * 16.0)) - 1.0)
#define UART_UBRR_ASYNC_DOUBLE(cpu,br)  ((cpu / (br * 8.0)) - 1.0)


/***** ENUMERATION ************************************************************/


/***** STRUCTURES *************************************************************/


/***** GLOBAL VARIABLES *******************************************************/
/* Circular buffer (needs to be global due to ISR) */
cbuf_t uart_cb_rx;
cbuf_t uart_cb_tx;
/* UART structure for function callbacks */
uart_isr_t uart_isr;


/***** LOCAL FUNCTION PROTOTYPES **********************************************/


/***** WRAPPER FUNCTIONS ******************************************************/


/***** INTERRUPT SERVICE ROUTINE (ISR) ****************************************/


/***** FUNCTIONS **************************************************************/
/*
 * Initialization of the UART
 */
void uart_init(void) {
    /* Set baudrate per default to 9600 */
    uart_set_baudrate(9600);
    
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
    cbuf_init(&uart_cb_rx);
    cbuf_init(&uart_cb_tx);
}


/*
 * Set the UART baudrate
 */
void uart_set_baudrate(uint32_t baudrate) {
    /* Set the baudrate */
#if UART_USE_2X
    UBRR0H = (uint8_t)((uint16_t)(UART_UBRR_ASYNC_DOUBLE(fuses_get_fcpu(),baudrate)) >> 8);
    UBRR0L = (uint8_t)((uint16_t)(UART_UBRR_ASYNC_DOUBLE(fuses_get_fcpu(),baudrate)) & 0x00FF);
#else
    UBRR0H = (uint8_t)((uint16_t)(UART_UBRR_ASYNC_NORMAL(fuses_get_fcpu(),baudrate)) >> 8);
    UBRR0L = (uint8_t)((uint16_t)(UART_UBRR_ASYNC_NORMAL(fuses_get_fcpu(),baudrate)) & 0x00FF);
#endif
}


/*
 * Enable UART interrupts
 */
void uart_interrupt_enable(void) {
    /* Enable "RX Complete Interrupt" */
    UCSR0B |= _BV(RXCIE0);
}


/*
 * Disable UART interrupts
 */
void uart_interrupt_disable(void) {
    /* Disable "RX Complete Interrupt" */
    UCSR0B &= ~_BV(RXCIE0);
    /* Disable "TX Complete Interrupt" */
    UCSR0B &= ~_BV(TXCIE0);
    /* Disable "USART Data Register Empty Interrupt" */
    UCSR0B &= ~_BV(UDRIE0);
}

/*
 * Set a UART RX callback function
 */
void uart_set_callback_rx(void (*callback)()) {
    /* Set the callback function */
    uart_isr.f_rx = callback;
}


/*
 * Set a UART TX callback function
 */
void uart_set_callback_tx(void (*callback)()) {
    /* Set the callback function */
    uart_isr.f_tx = callback;
}


/*
 * Set a UART EMPTY callback function
 */
void uart_set_callback_empty(void (*callback)()) {
    /* Set the callback function */
    uart_isr.f_empty = callback;
}


/**************************************/
/***** BLOCKING ***********************/
/**************************************/

/*
 * Write a character via UART
 */
void uart_putc(char c) {
    /* Wait for transmit buffer to be empty */
    while(!(UCSR0A & _BV(UDRE0)));
    /* Write byte to the output buffer */
    UDR0 = c;
}


/*
 * Write a string via UART
 */
void uart_puts(char* s) {
    /* Transmit the string character by character */
    while(*s) {
        uart_putc(*s++);
    }
}

/*
 * Write data to the UART in blocking mode
 */
int8_t uart_write_blocking(uint8_t* data, uint16_t len) {
    uint16_t i;
    /* Put the specified number of bytes in the TX buffer */
    for(i=0; i<len; i++) {
        /* Write the byte via UART */
        uart_putc(data[i]);
    }
    /* Return success */
    return UART_RET_OK;
}


/*
 * Read a character from UART
 */
uint8_t uart_getc(void) {
    /* Wait until reception is finished */
    while(!(UCSR0A & _BV(RXC0)));
    /* Return the received byte */
    return UDR0;
}


/*
 * Read a string from UART
 */
int8_t uart_gets(uint8_t* s, uint16_t len) {
    uint16_t index = 0;
    uint8_t temp;

    /* Read until string is finished or len is reached */
    do {
        /* Read next character */
        temp = uart_getc();
        /* Store character in string buffer */
        s[index++] = temp;
    } while((index < len) && (temp != '\n'));

    /* Make sure the string is null terminated */
    s[index] = '\0';
    
    /* Return success */
    return UART_RET_OK;
}


/**************************************/
/***** NON-BLOCKING *******************/
/**************************************/

/*
 * Write data to the UART in non-blocking mode (ISR)
 */
int8_t uart_write(uint8_t* data, uint16_t len) {
    uint16_t i;
    /* Put the specified number of bytes in the TX buffer */
    for(i=0; i<len; i++) {
        /* Try to push byte on the CB */
        if(cbuf_push(&uart_cb_tx,data[i]) != CBUF_RET_OK) {
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


/*
 * Read from the UART in non-blocking mode (ISR)
 */
uint16_t uart_read(uint8_t* data, uint16_t len) {
    uint16_t i=0;
    /* Get bytes from the RX buffer */
    for(i=0; i<len; i++) {
        uint8_t tmp;
        /* Try to pop one byte from the CB */
        int8_t ret = cbuf_pop(&uart_cb_rx, &tmp);
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


/*
 * Get the amount of byte currently in the RX buffer
 */
uint16_t uart_rx_buffer_cnt(void) {
    /* Return number of bytes in RX buffer */
    return uart_cb_rx.cnt;
}


/*
 * Get the amount of byte currently in the TX buffer
 */
uint16_t uart_tx_buffer_cnt(void) {
    /* Return number of bytes in TX buffer */
    return uart_cb_tx.cnt;
}


/*
 * Flush the UART RX buffer
 */
void uart_rx_flush(void) {
    /* Flush the CB */
    cbuf_flush(&uart_cb_rx);
}


/*
 * Flush the UART RX buffer
 */
void uart_tx_flush(void) {
    /* Flush the CB */
    cbuf_flush(&uart_cb_tx);
}


/***** INTERRUPT SERVICE ROUTINES (ISR) *******************************/
/*
 * RX Complete Interrupt
 */
ISR(USART0_RX_vect) {
    /* Push received byte in RX buffer */
    cbuf_push(&uart_cb_rx,UDR0);
    /* Check if a callback function was defined */
    if(uart_isr.f_rx != NULL) {
        /* Call function */
        uart_isr.f_rx();
    }
}


/*
 * TX Complete Interrupt
 */
ISR(USART0_TX_vect) {
    /* Check if a callback function was defined */
    if(uart_isr.f_tx != NULL) {
        /* Call function */
        uart_isr.f_tx();
    }
}


/*
 * USART Data Register Empty Interrupt
 */
ISR(USART0_UDRE_vect) {
    uint8_t tmp;
    int8_t ret;
    /* Try to pop one byte from the CB */
    ret = cbuf_pop(&uart_cb_tx, &tmp);
    /* Check if pop was successful */
    if(ret == CBUF_RET_OK) {
        /* Send byte via UART */
        UDR0 = tmp;
    } else {
        /* Clear receive interrupt */
        UCSR0B &= ~(_BV(UDRIE0));
        /* Check if a callback function was defined */
        if(uart_isr.f_empty != NULL) {
            /* Call function */
            uart_isr.f_empty();
        }
    }
}
