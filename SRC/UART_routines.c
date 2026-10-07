//**************************************************************
// UART routines for AVR ATmega8535
// F_CPU = 4 MHz
// Baud rate = 19200
// Format = 8N1
//
// RX is interrupt driven (ring buffer), TX is blocking.
//**************************************************************

#include "UART_routines.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <stdint.h>

#define UART_BAUD       19200UL
#define UART_UBRR_VALUE ((F_CPU / (16UL * UART_BAUD)) - 1UL)

/* Must be a power of two */
#define UART_RX_BUF_SIZE 32U
#define UART_RX_MASK     (UART_RX_BUF_SIZE - 1U)

static volatile uint8_t rx_buf[UART_RX_BUF_SIZE];
static volatile uint8_t rx_head;      // written by ISR
static volatile uint8_t rx_tail;      // read by main loop
static volatile uint8_t rx_overflow;  // count of lost bytes

//**************************************************
// UART0 initialize: 19200 8N1, RX interrupt enabled
//**************************************************
void uart0_init(void)
{
    // Disable UART while configuring
    UCSRB = 0x00;

    // Normal speed mode, U2X = 0
    UCSRA = 0x00;

    // Set baud rate
    UBRRH = (uint8_t)(UART_UBRR_VALUE >> 8);
    UBRRL = (uint8_t)(UART_UBRR_VALUE & 0xFF);

    // 8 data bits, no parity, 1 stop bit
    // For ATmega8535, URSEL must be 1 when writing UCSRC
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);

    rx_head = 0;
    rx_tail = 0;
    rx_overflow = 0;

    // Enable receiver, transmitter and RX complete interrupt
    UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
}

ISR(USART_RX_vect)
{
    uint8_t status = UCSRA;
    uint8_t data = UDR;
    uint8_t next = (uint8_t)((rx_head + 1U) & UART_RX_MASK);

    if (status & ((1 << FE) | (1 << DOR) | (1 << PE)))
    {
        // framing / overrun error: replace by a byte the parser rejects
        data = 0x00;
    }

    if (next == rx_tail)
    {
        rx_overflow++;      // buffer full, byte lost
        return;
    }

    rx_buf[rx_head] = data;
    rx_head = next;
}

//**************************************************
// Non-blocking read. Returns -1 if no data.
//**************************************************
int16_t uart_getc(void)
{
    uint8_t c;

    if (rx_head == rx_tail)
    {
        return -1;
    }

    c = rx_buf[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1U) & UART_RX_MASK);
    return c;
}

uint8_t uart_rx_overflow_take(void)
{
    uint8_t n;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        n = rx_overflow;
        rx_overflow = 0;
    }
    return n;
}

//**************************************************
// Receive a single byte (blocking)
//**************************************************
uint8_t receiveByte(void)
{
    int16_t c;

    while ((c = uart_getc()) < 0)
    {
        // Wait for incoming data
    }

    return (uint8_t)c;
}

//**************************************************
// Transmit a single byte
//**************************************************
void transmitByte(uint8_t data)
{
    while ((UCSRA & (1 << UDRE)) == 0)
    {
        // Wait until transmit buffer is empty
    }

    UDR = data;
}

//**************************************************
// Transmit hex format data
// dataType: CHAR, INT or LONG
// data: value to print
//**************************************************
void transmitHex(uint8_t dataType, uint32_t data)
{
    uint8_t count;
    uint8_t i;
    uint8_t temp;

    char dataString[] = "0x        ";

    switch (dataType)
    {
        case CHAR:
            count = 2;
            break;

        case INT:
            count = 4;
            break;

        case LONG:
            count = 8;
            break;

        default:
            transmitString("HEX?");
            return;
    }

    for (i = count; i > 0; i--)
    {
        temp = (uint8_t)(data & 0x0F);

        if (temp < 10)
        {
            dataString[i + 1] = (char)(temp + '0');
        }
        else
        {
            dataString[i + 1] = (char)(temp - 10 + 'A');
        }

        data >>= 4;
    }

    dataString[count + 2] = '\0';

    transmitString(dataString);
}

//**************************************************
// Transmit a string from Flash / Program Memory
// Example: transmitString_F(PSTR("Hello"));
//**************************************************
void transmitString_F(const char *string)
{
    char c;

    while ((c = (char)pgm_read_byte(string++)) != '\0')
    {
        transmitByte((uint8_t)c);
    }
}

//**************************************************
// Transmit a string from RAM
//**************************************************
void transmitString(const char *string)
{
    while (*string != '\0')
    {
        transmitByte((uint8_t)*string++);
    }
}
