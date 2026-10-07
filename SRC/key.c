//**********************************************************************
// Input / output pin setup
//**********************************************************************
#include "key.h"

// DDR = 0 - input  (PORT = 1 enables the internal pull-up)
//     = 1 - output
void init_pins(void)
{
    // PORTC: all inputs with pull-ups
    //   PC0/PC1 - TWI SCL/SDA (LCD), pull-ups are harmless for TWI
    //   PC2..PC4 - leak sensors (active LOW)
    //   PC5/PC6 - valve limit switches (active LOW)
    DDRC  = 0x00;
    PORTC = 0xFF;

    // PORTD:
    //   PD0/PD1 - UART RXD/TXD, left to the UART (not touched here)
    //   PD2..PD4 - outputs (as before)
    //   PD5 - OC1A PWM output (set in pwm_init)
    //   PD6/PD7 - buttons, inputs with pull-ups
    DDRD  |=  (uint8_t)((1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5));
    DDRD  &= (uint8_t)~((1 << PD6) | (1 << PD7));
    PORTD |=  (uint8_t)((1 << PD6) | (1 << PD7));
}
