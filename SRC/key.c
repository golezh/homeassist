//**********************************************************************
//Initialase buffer for button check
//*********************************************************************
#include "key.h"
// DDR = 0 - input; PORTX = 1; 10k to power +
//     = 1 - output
//   
void init_pins(void)
{
  DDR_OUT = 0x00;  // 0 - input
  PORT_OUT = 0xFF; // 1 pins to power via 10k resistor
  //DDR_OUT  |= 0b11111111;  // 1 - output
  //PORT_OUT |= 0x00; 
  DDR_KEY  |= 0b00111111;  // 0 - input
  PORT_KEY |= 0b00000000; // 1 pins to power via 10k resistor
  //  DDR_K &= ~(1<<SW0)|(1<<SW1)|(1<<SW2)|(1<<SW3);   /*Set pins on input*/
  //  KEY_PORT |= (1<<SW0)|(1<<SW1)|(1<<SW2)|(1<<SW3); /*Pull up resistance activate*/
}
	

