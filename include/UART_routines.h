#ifndef UART_ROUTINES_H
#define UART_ROUTINES_H

#include <stdint.h>

#define CHAR  1
#define INT   2
#define LONG  3

void uart0_init(void);

uint8_t receiveByte(void);
void transmitByte(uint8_t data);

void transmitHex(uint8_t dataType, uint32_t data);

void transmitString(const char *string);
void transmitString_F(const char *string);

#endif