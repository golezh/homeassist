#include "relay.h"

#include <avr/io.h>

// ------------------------------------------------------------
// Relay pin definitions
// ------------------------------------------------------------
#define RELAY1  PA4
#define RELAY2  PA3
#define RELAY3  PA2
#define RELAY4  PC6
#define RELAY5  PA0
#define RELAY6  PB0
#define RELAY7  PB1
#define RELAY8  PB2

typedef struct
{
    volatile uint8_t *ddr;
    volatile uint8_t *port;
    uint8_t pin;
} Relay_t;

static Relay_t relays[] =
{
    { &DDRA, &PORTA, RELAY1 },
    { &DDRA, &PORTA, RELAY2 },
    { &DDRA, &PORTA, RELAY3 },
    { &DDRC, &PORTC, RELAY4 },
    { &DDRA, &PORTA, RELAY5 },
    { &DDRB, &PORTB, RELAY6 },
    { &DDRB, &PORTB, RELAY7 },
    { &DDRB, &PORTB, RELAY8 }
};

#define RELAY_COUNT_INTERNAL ((uint8_t)(sizeof(relays) / sizeof(relays[0])))

uint8_t relay_count(void)
{
    return RELAY_COUNT_INTERNAL;
}

void relay_init(void)
{
    uint8_t i;

    for (i = 0; i < RELAY_COUNT_INTERNAL; i++)
    {
        // Set relay pin as output
        *relays[i].ddr |= (uint8_t)(1U << relays[i].pin);

        // Switch relay off initially
        relay_off(i);
    }
}

void relay_on(uint8_t index)
{
    if (index >= RELAY_COUNT_INTERNAL)
    {
        return;
    }

#if RELAY_ACTIVE_HIGH
    *relays[index].port |= (uint8_t)(1U << relays[index].pin);
#else
    *relays[index].port &= (uint8_t)~(1U << relays[index].pin);
#endif
}

void relay_off(uint8_t index)
{
    if (index >= RELAY_COUNT_INTERNAL)
    {
        return;
    }

#if RELAY_ACTIVE_HIGH
    *relays[index].port &= (uint8_t)~(1U << relays[index].pin);
#else
    *relays[index].port |= (uint8_t)(1U << relays[index].pin);
#endif
}

void relay_all_off(void)
{
    uint8_t i;

    for (i = 0; i < RELAY_COUNT_INTERNAL; i++)
    {
        relay_off(i);
    }
}

void relay_only(uint8_t index)
{
    relay_all_off();
    relay_on(index);
}