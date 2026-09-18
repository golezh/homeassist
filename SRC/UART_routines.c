//**************************************************************
// UART routines for AVR ATmega8535
// F_CPU = 4 MHz
// Baud rate = 19200
// Format = 8N1
//**************************************************************

#include "UART_routines.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>

#define UART_BAUD       19200UL
#define UART_UBRR_VALUE ((F_CPU / (16UL * UART_BAUD)) - 1UL)

//**************************************************
// UART0 initialize
// baud rate: 19200, F_CPU = 4 MHz
// char size: 8 bit
// parity: none
// stop bits: 1
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

    // Enable receiver and transmitter
    UCSRB = (1 << RXEN) | (1 << TXEN);
}

//**************************************************
// Receive a single byte
//**************************************************
uint8_t receiveByte(void)
{
    while ((UCSRA & (1 << RXC)) == 0)
    {
        // Wait for incoming data
    }

    return UDR;
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