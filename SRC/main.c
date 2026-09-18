//***************************************************************************
//  File........: main.c
//
//  Target(s)...: ATMega8535
//
//  Compiler....: WINAVR
//
//
//  Data........: 8.11.26
//
//***************************************************************************
#include <avr/io.h>
#include <avr\interrupt.h>       // Add the necessary ones
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include "UART_routines.h"
#include "ADC_routines.h"
#include <avr/interrupt.h>
#include "PWM.h"
#include "key.h"
#include "relay.h"
#include "timer.h"
#include <stdbool.h>

// #define CS_LOW	     PORT_OUT &= ~(1 << PC2)    			  /* CS=low */ 
// #define	CS_HIGH      PORT_OUT |=  (1 << PC2)		    	  /* CS=high */
/* #define LedON        PORT_KEY &=~(1<<PD5); //0
#define LedOFF       PORT_KEY |=(1<<PD5);  //1 */

#define F_CPU 4000000UL		//freq 4 MHz
extern volatile unsigned char temperature[7];
extern volatile unsigned char voltage[7];
uint32_t previous_time = 0;
volatile enum SystemState active_state = STATE_IDLE;
enum ValveState
{
    VALVE_CLOSED = 0,
    VALVE_OPENED,
    VALVE_OPENING,
    VALVE_CLOSING
} valve_state = VALVE_CLOSED;
enum SystemState
{
    STATE_IDLE = 0,
    STATE_START,
    STATE_VALVE_OPEN,
    STATE_VALVE_OPENING,
    STATE_VALVE_CLOSE,
    STATE_VALVE_CLOSING,
    STATE_LEAK_DETECTED,
    STATE_LEAK_CLEARED,
    STATE_ERROR,
    STATE_OPEN,
    STATE_CLOSE
} system_state = STATE_IDLE;

struct Button_t
{
    volatile uint8_t Open;
    volatile uint8_t Close;
    volatile uint8_t Action;
    volatile uint8_t current_relay; // 0 - valve open, 1 - valve close used for relay off set
} Operation_t;

/* #define SENSOR0 PC0 */
#define SENSOR1 PC1
#define SENSOR2 PC2
#define SENSOR3 PC3
#define SENSOR4 PC4
#define SENSOR5 PC5
#define SENSOR6 PC6
#define SENSOR7 PC7 
struct Sensor_t
{
    volatile uint8_t pin;
    volatile uint8_t state;
    volatile uint8_t leak_active;
} Sensors[] = {
/*     {SENSOR0, 0}, */
    {SENSOR1, 0, 0},
    {SENSOR2, 0, 0},
    {SENSOR3, 0, 0},
    {SENSOR4, 0, 0},
    {SENSOR5, 0, 0},
};
struct SensorValve_t {
    volatile uint8_t pin;
    enum ValveState valve_state;
}SensorsValve[] = {
    {SENSOR6, VALVE_CLOSED},
    {SENSOR7, VALVE_CLOSED}
};


#define SENSOR_COUNT ((uint8_t)(sizeof(Sensors) / sizeof(Sensors[0])))

static inline uint8_t button_click_event(volatile uint8_t *pin_reg, uint8_t pin)
{
    uint8_t mask = (uint8_t)(1U << pin);

    if ((*pin_reg & mask) != 0)
    {
        return 0;
    }

    _delay_ms(10);

    if ((*pin_reg & mask) != 0)
    {
        return 0;
    }

    while ((*pin_reg & mask) == 0)
    {
    }

    _delay_ms(10);

    return 1;
}

void port_init(void)
{
  PORTA = 0x00;
  DDRA  = 0x00;
}

//call this routine to initialize all peripherals
void init_devices(void)
{
  cli();        //all interrupts disabled
  port_init();
  uart0_init();
  //pwm_init();
  //ADC_init();
  
  MCUCR = 0x00;
  GICR  = 0x00;
  TIMSK = 0x00; //timer interrupt sources
                //all peripherals are now initialized
  timer1_init_1ms();
}
bool read_sensors(volatile uint8_t *pin_reg, uint8_t pin)
{
    uint8_t mask = (uint8_t)(1U << pin);

    return ((*pin_reg & mask) == 0);   // true = active LOW
}

void valve_control(uint8_t relay_index, uint8_t action)
{
    if (action == 1) // Open
    {
        relay_on(relay_index);
        transmitString_F(PSTR("\n\rRelay "));
        transmitHex(CHAR, relay_index);
        transmitString_F(PSTR(" is ON."));
    }
    else if (action == 0) // Close
    {
        relay_off(relay_index);
        transmitString_F(PSTR("\n\rRelay "));
        transmitHex(CHAR, relay_index);
        transmitString_F(PSTR(" is OFF."));
    }
}
void check_sensors(volatile uint8_t *pin_reg)
{
    if((*pin_reg & 0x1) == 0)
        transmitString_F(PSTR("\n\r1-st sensor is active."));
    /*Kitchen*/
    if((*pin_reg & 0x2) == 0){
        transmitString_F(PSTR("\n\r2-th sensor is active."));
        Sensors[0].leak_active = 1;
        active_state = STATE_LEAK_DETECTED;
    }else{
        Sensors[0].leak_active = 0;
        active_state = STATE_LEAK_CLEARED;
    }
    /*Bathroom 1*/
    if((*pin_reg & 0x4) == 0){
        transmitString_F(PSTR("\n\r3-rd sensor is active."));
        Sensors[1].leak_active = 1;
        active_state = STATE_LEAK_DETECTED;
    }else{
        Sensors[1].leak_active = 0;
    }
    /*Bathroom 2*/
    if((*pin_reg & 0x8) == 0){
        transmitString_F(PSTR("\n\r4-th sensor is active."));
        Sensors[2].leak_active = 1;
        active_state = STATE_LEAK_DETECTED;
    }else{
        Sensors[2].leak_active = 0;
    }
    /*not using this sensor for now*/
    if((*pin_reg & 0x10) == 0){
        transmitString_F(PSTR("\n\r5-th sensor is active."));
        Sensors[2].leak_active = 1;
        active_state = STATE_LEAK_DETECTED;
    }else{
        Sensors[2].leak_active = 0;
    }
    if((*pin_reg & 0x20) == 0){
        transmitString_F(PSTR("\n\rValve is opened."));
        active_state = STATE_OPEN;
        SensorsValve[0].valve_state = VALVE_OPENED; //indicate that the valve is opened
    }
    if((*pin_reg & 0x40) == 0){
        transmitString_F(PSTR("\n\r7-th sensor is active."));
        active_state = STATE_CLOSE;
        SensorsValve[0].valve_state = VALVE_CLOSED; //indicate that the valve is closed
    }
    
}
void check_leakage(){
    if(Sensors[1].leak_active || Sensors[2].leak_active || Sensors[3].leak_active || Sensors[4].leak_active){
        transmitString_F(PSTR("\n\rLeakage detected!"));
        if(SensorsValve[0].valve_state == VALVE_OPENED){
            transmitString_F(PSTR("\n\rClosing the valve."));
            active_state = STATE_CLOSE;
        }
    }else{
        transmitString_F(PSTR("\n\rNo leakage detected."));
        active_state = STATE_LEAK_CLEARED;
    }
}
void valve_open(){
    relay_on(1); // Open the valve
    Operation_t.current_relay = 1; // Set current relay to 1 for valve open
    SensorsValve[0].valve_state = VALVE_OPENING; //indicate that the valve start opening
    Operation_t.Action = 1; //indicate that the valve is opening
    transmitString_F(PSTR("\n\rOpening the valve."));
}
void valve_close(){
    relay_on(2); // Close the valve
    Operation_t.current_relay = 2; // Set current relay to 2 for valve close
    SensorsValve[0].valve_state = VALVE_CLOSING; //indicate that the valve start closing
    Operation_t.Action = 1; //indicate that the valve is closing
    transmitString_F(PSTR("\n\rClosing the valve."));
}
void call_state_machine(){
    switch (active_state)
    {
    case STATE_START:
        transmitString_F(PSTR("\n\rSystem started!"));
        if(SensorsValve[0].valve_state == VALVE_OPENED){
            transmitString_F(PSTR("\n\rValve is opened!"));
            active_state = STATE_OPEN;
        }
        active_state = STATE_IDLE;
        break;
    case STATE_IDLE:
        /* code */
        break;
    case STATE_LEAK_DETECTED:
        transmitString_F(PSTR("\n\rLeak detected!"));
        check_leakage();
        active_state = STATE_IDLE;
        break;
    case STATE_LEAK_CLEARED:
        transmitString_F(PSTR("\n\rLeak cleared!"));
        valve_open();
        active_state = STATE_IDLE;
        break;
    case STATE_OPEN:
        transmitString_F(PSTR("\n\rValve is opened!"));
        if(Operation_t.Action == 1){
            relay_off(Operation_t.current_relay); // Turn off the relay after closing the valve
            Operation_t.Action = 0; // Reset action after opening the valve
        }else{
         active_state = STATE_IDLE;
        }
        break;

    case STATE_CLOSE:
        transmitString_F(PSTR("\n\rValve is closed!"));
        if(Operation_t.Action == 1){
            relay_off(Operation_t.current_relay); // Turn off the relay after closing the valve
            Operation_t.Action = 0; // Reset action after closing the valve 
        }else{
         active_state = STATE_IDLE;
        }
        break;
    case STATE_VALVE_OPENING:
        transmitString_F(PSTR("\n\rValve is opening!"));
        if(Operation_t.Action == 1){
            relay_off(Operation_t.current_relay); // Turn off the relay after closing the valve
            Operation_t.Action = 0; // Reset action after opening the valve
        }else{
         active_state = STATE_IDLE;
        }
        break;
    case STATE_VALVE_CLOSING:
        transmitString_F(PSTR("\n\rValve is closing!"));
        if(Operation_t.Action == 1){
            relay_off(Operation_t.current_relay); // Turn off the relay after closing the valve
            Operation_t.Action = 0; // Reset action after closing the valve 
        }else{
         active_state = STATE_IDLE;
        }
    case STATE_ERROR:
        transmitString_F(PSTR("\n\rError detected!"));
        break;

    default:
        break;
    }
}
int main( void )
{
  init_devices();
  init_pins();
  relay_init();
  transmitString_F (PSTR("\n\r\n\r****************************************************"));
  transmitString_F (PSTR("\n\r         Leak Detector is started. V.0.1.0              "));
  transmitString_F (PSTR("\n\r****************************************************\n\r"));
  //Enable Global Interrupts
  sei();
  uint8_t current_relay = 0;
  bool sensor_active = false;
  Operation_t.Open = 0;
  Operation_t.Close = 0;
  relay_only(current_relay);
  uint8_t sensor_index = 0;
  DDRC  = 0x00; // 0 - input
  PORTC = 0xFF; // 1 pins to power via 10k resistor

  while (1)
  {

    if ((millis() - previous_time) >= 200)
    {
        previous_time = millis();
        check_sensors(&PINC);
        call_state_machine();
    }
  }      
  
  return 0;
}
#if 0
    if (button_click_event(&PIND, PD6) && Operation_t.Open == 0)
    {
      current_relay++;
      if (current_relay == 2)
      {
          current_relay = 0;
      }
      relay_only(current_relay);
      transmitString_F(PSTR("\n\rRelay changed to: "));
      transmitHex(CHAR, current_relay);
      Operation_t.Open = 1;
      Operation_t.Close = 0;
      Operation_t.Action = 1;

    }

    if (button_click_event(&PIND, PD7) && Operation_t.Close == 0)
    {
      relay_off(current_relay);
      transmitString_F(PSTR("\n\rAll relays turned off."));
      Operation_t.Open = 0;
      Operation_t.Close = 1;
      Operation_t.Action = 0;
    }
#endif