/*!
 * @brief   ASN(x) UART library -- source file
 *
 * Library to support the use of the UART (blocking and non-blocking).
 *
 * @file    /_asnx_lib_/uart/uart.c
 * @author  Dominik Widhalm
 * @version 1.2.2
 * @date    2022/01/24
 *
 * @todo    *) Add timeout to "write/read" functions (e.g., blocking)
 */


/***** INCLUDES *******************************************************/
#include "uart.h"


/***** DEFINES ********************************************************/
/*! Calculate the BAUD register values */
#if UART_USE_2X
#  define UART_UBRR(br)         ((F_CPU / (br * 8.0)) - 1.0)
#else
#  define UART_UBRR(br)         ((F_CPU / (br * 16.0)) - 1.0)
#endif


/***** GLOBAL VARIABLES ***********************************************/
/*** UART0 ***/
#if UART0_ENABLE_INTERRUPT
/* Circular buffers (need to be global due to ISR) */
cbuf_t uart0_cb_rx;         /**< UART0 receive buffer */
cbuf_t uart0_cb_tx;         /**< UART0 transmit buffer */
/* UART structures for function callbacks */
uart_isr_t uart0_isr;       /**< UART0 isr data structure */
#endif
/*** UART1 ***/
#if UART1_ENABLE_INTERRUPT
/* Circular buffers (need to be global due to ISR) */
cbuf_t uart1_cb_rx;         /**< UART1 receive buffer */
cbuf_t uart1_cb_tx;         /**< UART1 transmit buffer */
/* UART structures for function callbacks */
uart_isr_t uart1_isr;       /**< UART1 isr data structure */
#endif


/***** FUNCTIONS ******************************************************/
/*!
 * Initialize the UART0 interface.
 *
 * @param[in]   baudrate    The baud rate to be set.
 */
void uart0_init(uint32_t baudrate) {
#if UART_USE_2X
    /* Enable U2X mode */
    UCSR0A |= _BV(U2X0);
#else
    /* Disable U2X mode */
    UCSR0A &= ~_BV(U2X0);
#endif
    /* Set baudrate */
    UBRR0H = (uint8_t)((uint16_t)(UART_UBRR(baudrate)) >> 8);
    UBRR0L = (uint8_t)((uint16_t)(UART_UBRR(baudrate)) & 0x00FF);
    /* Set the framing (8N1) */
    UCSR0C |= _BV(UCSZ00) | _BV(UCSZ01);
    /* Enable RX and TX */
    UCSR0B |= _BV(RXEN0) | _BV(TXEN0);

#if UART0_ENABLE_INTERRUPT
    /* Initialize rx/tx buffer */
    cbuf_init(&uart0_cb_rx);
    cbuf_init(&uart0_cb_tx);
#endif
}


/*!
 * Initialize the UART1 interface.
 *
 * @param[in]   baudrate    The baud rate to be set.
 */
void uart1_init(uint32_t baudrate) {
#if UART_USE_2X
    /* Enable U2X mode */
    UCSR1A |= _BV(U2X1);
#else
    /* Disable U2X mode */
    UCSR1A &= ~_BV(U2X1);
#endif
    /* Set baudrate */
    UBRR1H = (uint8_t)((uint16_t)(UART_UBRR(baudrate)) >> 8);
    UBRR1L = (uint8_t)((uint16_t)(UART_UBRR(baudrate)) & 0x00FF);
    /* Set the framing (8N1) */
    UCSR1C |= _BV(UCSZ10) | _BV(UCSZ11);
    /* Enable RX and TX */
    UCSR1B |= _BV(RXEN1) | _BV(TXEN1);

#if UART1_ENABLE_INTERRUPT
    /* Initialize rx/tx buffer */
    cbuf_init(&uart1_cb_rx);
    cbuf_init(&uart1_cb_tx);
#endif
}


/*!
 * Enable the UART0 interface.
 */
void uart0_enable(void) {
    /* Enable RX and TX */
    UCSR0B |= _BV(RXEN0) | _BV(TXEN0);
}

/*!
 * Disable the UART0 interface.
 */
void uart0_disable(void) {
    /* Disable RX and TX */
    UCSR0B &= ~(_BV(RXEN0) | _BV(TXEN0));
}


/*!
 * Enable the UART1 interface.
 */
void uart1_enable(void) {
    /* Enable RX and TX */
    UCSR1B |= _BV(RXEN1) | _BV(TXEN1);
}


/*!
 * Disable the UART1 interface.
 */
void uart1_disable(void) {
    /* Disable RX and TX */
    UCSR1B &= ~(_BV(RXEN1) | _BV(TXEN1));
}


/*!
 * Set the BAUD rate of UART0.
 *
 * @param[in]   baudrate    The baud rate to be set.
 */
void uart0_set_baudrate(uint32_t baudrate) {
    /* Set the baudrate */
    UBRR0H = (uint8_t)((uint16_t)(UART_UBRR(baudrate)) >> 8);
    UBRR0L = (uint8_t)((uint16_t)(UART_UBRR(baudrate)) & 0x00FF);
}


/*!
 * Set the BAUD rate of UART1.
 *
 * @param[in]   baudrate    The baud rate to be set.
 */
void uart1_set_baudrate(uint32_t baudrate) {
    /* Set the baudrate */
    UBRR1H = (uint8_t)((uint16_t)(UART_UBRR(baudrate)) >> 8);
    UBRR1L = (uint8_t)((uint16_t)(UART_UBRR(baudrate)) & 0x00FF);
}


#if UART0_ENABLE_INTERRUPT
/*!
 * Enable the UART0 receive complete interrupt.
 */
void uart0_interrupt_enable(void) {
    /* Enable "RX Complete Interrupt" */
    UCSR0B |= _BV(RXCIE0);
}


/*!
 * Disable all UART0 interrupts.
 */
void uart0_interrupt_disable(void) {
    /* Disable "RX Complete Interrupt" */
    UCSR0B &= ~_BV(RXCIE0);
    /* Disable "TX Complete Interrupt" */
    UCSR0B &= ~_BV(TXCIE0);
    /* Disable "USART Data Register Empty Interrupt" */
    UCSR0B &= ~_BV(UDRIE0);
}


/*!
 * Set a UART0 RX callback function.
 *
 * @param[in]   callback    Callback function pointer
 */
void uart0_set_callback_rx(void (*callback)(void)) {
    /* Set the callback function */
    uart0_isr.f_rx = callback;
}


/*!
 * Set a UART0 TX callback function.
 *
 * @param[in]   callback    Callback function pointer
 */
void uart0_set_callback_tx(void (*callback)(void)) {
    /* Set the callback function */
    uart0_isr.f_tx = callback;
}


/*!
 * Set a UART0 EMPTY callback function.
 *
 * @param[in]   callback    Callback function pointer
 */
void uart0_set_callback_empty(void (*callback)(void)) {
    /* Set the callback function */
    uart0_isr.f_empty = callback;
}
#endif


#if UART1_ENABLE_INTERRUPT
/*!
 * Enable the UART1 receive complete interrupt.
 */
void uart1_interrupt_enable(void) {
    /* Enable "RX Complete Interrupt" */
    UCSR1B |= _BV(RXCIE1);
}


/*!
 * Disable all UART1 interrupts.
 */
void uart1_interrupt_disable(void) {
    /* Disable "RX Complete Interrupt" */
    UCSR1B &= ~_BV(RXCIE1);
    /* Disable "TX Complete Interrupt" */
    UCSR1B &= ~_BV(TXCIE1);
    /* Disable "USART Data Register Empty Interrupt" */
    UCSR1B &= ~_BV(UDRIE1);
}


/*!
 * Set a UART1 RX callback function.
 *
 * @param[in]   callback    Callback function pointer
 */
void uart1_set_callback_rx(void (*callback)(void)) {
    /* Set the callback function */
    uart1_isr.f_rx = callback;
}


/*!
 * Set a UART1 TX callback function.
 *
 * @param[in]   callback    Callback function pointer
 */
void uart1_set_callback_tx(void (*callback)(void)) {
    /* Set the callback function */
    uart1_isr.f_tx = callback;
}


/*!
 * Set a UART1 EMPTY callback function.
 *
 * @param[in]   callback    Callback function pointer
 */
void uart1_set_callback_empty(void (*callback)(void)) {
    /* Set the callback function */
    uart1_isr.f_empty = callback;
}
#endif


/*!
 * Transmit a character via UART0 (blocking).
 *
 * @param[in]   char        Character to be transmitted
 */
void uart0_write_char(char character) {
    /* Wait for transmit buffer to be empty */
    while(!(UCSR0A & _BV(UDRE0)));
    /* Write character to the output buffer */
    UDR0 = character;
}


/*!
 * Transmit a character via UART1 (blocking).
 *
 * @param[in]   char        Character to be transmitted
 */
void uart1_write_char(char character) {
    /* Wait for transmit buffer to be empty */
    while(!(UCSR1A & _BV(UDRE1)));
    /* Write character to the output buffer */
    UDR1 = character;
}


/*!
 * Transmit a byte via UART0 (blocking).
 *
 * @param[in]   byte        Byte to be transmitted
 */
void uart0_write_byte(uint8_t byte) {
    /* Wait for transmit buffer to be empty */
    while(!(UCSR0A & _BV(UDRE0)));
    /* Write byte to the output buffer */
    UDR0 = byte;
}


/*!
 * Transmit a byte via UART1 (blocking).
 *
 * @param[in]   byte        Byte to be transmitted
 */
void uart1_write_byte(uint8_t byte) {
    /* Wait for transmit buffer to be empty */
    while(!(UCSR1A & _BV(UDRE1)));
    /* Write byte to the output buffer */
    UDR1 = byte;
}


/*!
 * Transmit a number of bytes via UART0 (blocking).
 *
 * @param[in]   data        Pointer to the data array
 * @param[in]   len         Number of bytes to be transmitted
 * @return      OK in case of successful
 */
int8_t uart0_write_block(uint8_t* data, uint16_t len) {
    uint16_t i;
    /* Put the specified number of bytes in the TX buffer */
    for(i=0; i<len; i++) {
        /* Write the byte via UART */
        uart0_write_byte(data[i]);
    }
    return UART_RET_OK;
}


/*!
 * Transmit a number of bytes via UART1 (blocking).
 *
 * @param[in]   data        Pointer to the data array
 * @param[in]   len         Number of bytes to be transmitted
 * @return      OK in case of successful
 */
int8_t uart1_write_block(uint8_t* data, uint16_t len) {
    uint16_t i;
    /* Put the specified number of bytes in the TX buffer */
    for(i=0; i<len; i++) {
        /* Write the byte via UART */
        uart1_write_byte(data[i]);
    }
    return UART_RET_OK;
}


/*!
 * Receive a byte from UART0 (blocking).
 *
 * @return      Byte read
 */
uint8_t uart0_read_byte(void) {
    /* Wait until reception is finished */
    while(!(UCSR0A & _BV(RXC0)));
    /* Return the received byte */
    return UDR0;
}


/*!
 * Receive a byte from UART1 (blocking).
 *
 * @return      Byte read
 */
uint8_t uart1_read_byte(void) {
    /* Wait until reception is finished */
    while(!(UCSR1A & _BV(RXC1)));
    /* Return the received byte */
    return UDR1;
}


/*!
 * Receive a string from UART0 (blocking) up to a certain length.
 *
 * @param[out]  data        Data to be received.
 * @param[in]   len         Maximum number of bytes to be received
 * @return      OK in case of successful
 */
int8_t uart0_read_block(uint8_t* data, uint16_t len) {
    uint16_t index = 0;
    uint8_t temp;
    /* Read until string is finished or len is reached */
    do {
        /* Read next character */
        temp = uart0_read_byte();
        /* Store character in string buffer */
        data[index++] = temp;
    } while(index < len);
    /* Return success */
    return UART_RET_OK;
}


/*!
 * Receive a string from UART1 (blocking) up to a certain length.
 *
 * @param[out]  data        Data to be received.
 * @param[in]   len         Maximum number of bytes to be received
 * @return      OK in case of successful
 */
int8_t uart1_read_block(uint8_t* data, uint16_t len) {
    uint16_t index = 0;
    uint8_t temp;
    /* Read until string is finished or len is reached */
    do {
        /* Read next character */
        temp = uart1_read_byte();
        /* Store character in string buffer */
        data[index++] = temp;
    } while(index < len);
    /* Return success */
    return UART_RET_OK;
}


#if UART0_ENABLE_INTERRUPT
/*!
 * Write a byte to the UART0 TX buffer (non-blocking ISR mode).
 *
 * @param[in]   byte        Byte to be written
 * @return      OK in case of successful
 */
int8_t uart0_put_byte(uint8_t byte) {
    /* Try to push byte on the CB */
    if(cbuf_push(&uart0_cb_tx,byte) != CBUF_RET_OK) {
        /* Push didn't work ...*/
        return UART_RET_ERROR;
    } else {
        /* Activate buffer empty interrupt */
        UCSR0B |= _BV(UDRIE0);
    }
    /* Return success */
    return UART_RET_OK;
}


/*!
 * Read a byte from the UART0 RX buffer (non-blocking ISR mode).
 *
 * @param[out]  byte        Byte to be read
 * @return      OK in case of successful
 */
int8_t uart0_pop_byte(uint8_t* byte) {
    /* Check if pop was successful */
    if(cbuf_pop(&uart0_cb_rx, byte) == CBUF_RET_OK) {
        /* Return success */
        return UART_RET_OK;
    } else  {
        /* Pop didn't work */
        *byte = 0x00;
        /* Return fail */
        return UART_RET_ERROR;
    }
}


/*!
 * Write data to the UART0 TX buffer (non-blocking ISR mode).
 *
 * @param[in]   data        Pointer to the data array to be written
 * @param[in]   len         Number of bytes to be written
 * @return      OK in case of successful
 */
int8_t uart0_put_block(uint8_t* data, uint16_t len) {
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


/*!
 * Read data from the UART0 RX buffer (non-blocking ISR mode).
 *
 * @param[out]  data        Pointer to the data array to be read
 * @param[in]   len         Maximum number of bytes to be read
 * @param[in]   cnt         Number of bytes read
 * @return      OK in case of successful
 */
int8_t uart0_pop_block(uint8_t* data, uint16_t len, uint8_t* cnt) {
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
        } else if(ret == CBUF_RET_EMPTY) {
            /* Nothing left to pop */
            break;
        }
    }
    /* Return number of bytes read */
    *cnt = i;
    /* Return success */
    return UART_RET_OK;
}


/*!
 * Get the amount of byte currently in the UART0 RX buffer.
 *
 * @return      Number of bytes in the UART0 RX buffer
 */
uint8_t uart0_rx_buffer_cnt(void) {
    /* Return number of bytes in RX buffer */
    return uart0_cb_rx.cnt;
}


/*!
 * Get the amount of byte currently in the UART0 TX buffer.
 *
 * @return      Number of bytes in the UART0 TX buffer
 */
uint8_t uart0_tx_buffer_cnt(void) {
    /* Return number of bytes in TX buffer */
    return uart0_cb_tx.cnt;
}


/*!
 * Flush the UART0 RX buffer.
 */
void uart0_rx_cb_flush(void) {
    /* Flush the CB */
    cbuf_flush(&uart0_cb_rx);
}


/*!
 * Flush the UART0 TX buffer.
 */
void uart0_tx_cb_flush(void) {
    /* Flush the CB */
    cbuf_flush(&uart0_cb_tx);
}
#endif


#if UART1_ENABLE_INTERRUPT
/*!
 * Write a byte to the UART1 TX buffer (non-blocking ISR mode).
 *
 * @param[in]   byte        Byte to be written
 * @return      OK in case of successful
 */
int8_t uart1_put_byte(uint8_t byte) {
    /* Try to push byte on the CB */
    if(cbuf_push(&uart1_cb_tx,byte) != CBUF_RET_OK) {
        /* Push didn't work ...*/
        return UART_RET_ERROR;
    } else {
        /* Activate buffer empty interrupt */
        UCSR1B |= _BV(UDRIE1);
    }
    /* Return success */
    return UART_RET_OK;
}


/*!
 * Read a byte from the UART1 RX buffer (non-blocking ISR mode).
 *
 * @param[out]  byte        Byte to be read
 * @return      OK in case of successful
 */
int8_t uart1_pop_byte(uint8_t* byte) {
    /* Check if pop was successful */
    if(cbuf_pop(&uart1_cb_rx, byte) == CBUF_RET_OK) {
        /* Return success */
        return UART_RET_OK;
    } else  {
        /* Pop didn't work */
        *byte = 0x00;
        /* Return fail */
        return UART_RET_ERROR;
    }
}


/*!
 * Write data to the UART1 TX buffer (non-blocking ISR mode).
 *
 * @param[in]   data        Pointer to the data array to be written
 * @param[in]   len         Number of bytes to be written
 * @return      OK in case of successful
 */
int8_t uart1_put_block(uint8_t* data, uint16_t len) {
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


/*!
 * Read data from the UART1 RX buffer (non-blocking ISR mode).
 *
 * @param[out]  data        Pointer to the data array to be read
 * @param[in]   len         Maximum number of bytes to be read
 * @param[in]   cnt         Number of bytes read
 * @return      OK in case of successful
 */
int8_t uart1_pop_block(uint8_t* data, uint16_t len, uint8_t* cnt) {
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
        } else if(ret == CBUF_RET_EMPTY) {
            /* Nothing left to pop */
            break;
        }
    }
    /* Return number of bytes read */
    *cnt = i;
    /* Return success */
    return UART_RET_OK;
}


/*!
 * Get the amount of byte currently in the UART1 RX buffer.
 *
 * @return      Number of bytes in the UART1 RX buffer
 */
uint8_t uart1_rx_buffer_cnt(void) {
    /* Return number of bytes in RX buffer */
    return uart1_cb_rx.cnt;
}


/*!
 * Get the amount of byte currently in the UART1 TX buffer.
 *
 * @return      Number of bytes in the UART1 TX buffer
 */
uint8_t uart1_tx_buffer_cnt(void) {
    /* Return number of bytes in TX buffer */
    return uart1_cb_tx.cnt;
}


/*!
 * Flush the UART1 RX buffer.
 */
void uart1_rx_cb_flush(void) {
    /* Flush the CB */
    cbuf_flush(&uart1_cb_rx);
}


/*!
 * Flush the UART1 TX buffer.
 */
void uart1_tx_cb_flush(void) {
    /* Flush the CB */
    cbuf_flush(&uart1_cb_tx);
}
#endif


#if UART0_ENABLE_INTERRUPT
/*!
 * UART0 RX complete interrupt.
 */
ISR(USART0_RX_vect) {
    /* Push received byte in RX buffer */
    cbuf_push(&uart0_cb_rx,UDR0);
    /* Check if a callback function was defined */
    if(uart0_isr.f_rx != NULL) {
        /* Call function */
        uart0_isr.f_rx();
    }
}


/*!
 * UART0 TX complete interrupt.
 */
ISR(USART0_TX_vect) {
    /* Check if a callback function was defined */
    if(uart0_isr.f_tx != NULL) {
        /* Call function */
        uart0_isr.f_tx();
    }
}


/*!
 * UART0 data register empty interrupt.
 */
ISR(USART0_UDRE_vect) {
    uint8_t tmp;
    int8_t ret;
    /* Try to pop one byte from the CB */
    ret = cbuf_pop(&uart0_cb_tx, &tmp);
    /* Check if pop was successful */
    if(ret == CBUF_RET_OK) {
        /* Send byte via UART */
        UDR0 = tmp;
    } else if(ret == CBUF_RET_EMPTY) {
        /* Clear receive interrupt */
        UCSR0B &= ~(_BV(UDRIE0));
        /* Check if a callback function was defined */
        if(uart0_isr.f_empty != NULL) {
            /* Call function */
            uart0_isr.f_empty();
        }
    }
}
#endif


#if UART1_ENABLE_INTERRUPT
/*!
 * UART1 RX complete interrupt.
 */
ISR(USART1_RX_vect) {
    /* Push received byte in RX buffer */
    cbuf_push(&uart1_cb_rx,UDR1);
    /* Check if a callback function was defined */
    if(uart1_isr.f_rx != NULL) {
        /* Call function */
        uart1_isr.f_rx();
    }
}


/*!
 * UART1 TX complete interrupt.
 */
ISR(USART1_TX_vect) {
    /* Check if a callback function was defined */
    if(uart1_isr.f_tx != NULL) {
        /* Call function */
        uart1_isr.f_tx();
    }
}


/*!
 * UART1 data register empty interrupt.
 */
ISR(USART1_UDRE_vect) {
    uint8_t tmp;
    int8_t ret;
    /* Try to pop one byte from the CB */
    ret = cbuf_pop(&uart1_cb_tx, &tmp);
    /* Check if pop was successful */
    if(ret == CBUF_RET_OK) {
        /* Send byte via UART */
        UDR1 = tmp;
    } else if(ret == CBUF_RET_EMPTY) {
        /* Clear receive interrupt */
        UCSR1B &= ~(_BV(UDRIE1));
        /* Check if a callback function was defined */
        if(uart1_isr.f_empty != NULL) {
            /* Call function */
            uart1_isr.f_empty();
        }
    }
}
#endif
