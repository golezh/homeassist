
#ifndef KEY_LIB_H
#define KEY_LIB_H

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#define PORT_KEY PORTD
#define DDR_KEY DDRD
//#define PIN_K PINB
#define DDR_OUT DDRC
#define PORT_OUT PORTC

//#define SW0 PB0
//#define SW1 PB3
//#define SW2 PB2
//#define SW3 PB1

void init_pins(void);



#endif
