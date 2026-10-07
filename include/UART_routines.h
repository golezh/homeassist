#ifndef UART_ROUTINES_H
#define UART_ROUTINES_H

#include <stdint.h>

#define CHAR  1
#define INT   2
#define LONG  3

void uart0_init(void);

int16_t uart_getc(void);             // non-blocking, -1 = no data
uint8_t uart_rx_overflow_take(void); // lost RX bytes since last call
uint8_t receiveByte(void);           // blocking
void transmitByte(uint8_t data);

void transmitHex(uint8_t dataType, uint32_t data);

void transmitString(const char *string);
void transmitString_F(const char *string);

#endif
