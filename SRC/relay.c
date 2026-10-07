#include "relay.h"

#include <avr/io.h>
#include <avr/pgmspace.h>

// ------------------------------------------------------------
// Relay pin definitions
//
// NOTE: RELAY4 used to be on PC6, but PC6 is the "valve closed" limit
// switch input. Driving it as an output shorts the switch, so that
// relay was removed from the table. If it is ever needed, move it to a
// free pin and add it back here.
// ------------------------------------------------------------
#define RELAY1  PA4     // index 0 - valve CLOSE
#define RELAY2  PA3     // index 1 - valve OPEN
#define RELAY3  PA2
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

/* Table lives in flash to save RAM */
static const Relay_t relays[] PROGMEM =
{
    { &DDRA, &PORTA, RELAY1 },
    { &DDRA, &PORTA, RELAY2 },
    { &DDRA, &PORTA, RELAY3 },
    { &DDRA, &PORTA, RELAY5 },
    { &DDRB, &PORTB, RELAY6 },
    { &DDRB, &PORTB, RELAY7 },
    { &DDRB, &PORTB, RELAY8 }
};

#define RELAY_COUNT_INTERNAL ((uint8_t)(sizeof(relays) / sizeof(relays[0])))

static volatile uint8_t *relay_port(uint8_t i)
{
    return (volatile uint8_t *)pgm_read_word(&relays[i].port);
}

static volatile uint8_t *relay_ddr(uint8_t i)
{
    return (volatile uint8_t *)pgm_read_word(&relays[i].ddr);
}

static uint8_t relay_bit(uint8_t i)
{
    return (uint8_t)(1U << pgm_read_byte(&relays[i].pin));
}

uint8_t relay_count(void)
{
    return RELAY_COUNT_INTERNAL;
}

void relay_init(void)
{
    uint8_t i;

    for (i = 0; i < RELAY_COUNT_INTERNAL; i++)
    {
        // First set the "off" level in PORT, only then switch the pin to
        // output. Otherwise an active-LOW relay clicks on for a moment.
        relay_off(i);
        *relay_ddr(i) |= relay_bit(i);
    }
}

void relay_on(uint8_t index)
{
    if (index >= RELAY_COUNT_INTERNAL)
    {
        return;
    }

#if RELAY_ACTIVE_HIGH
    *relay_port(index) |= relay_bit(index);
#else
    *relay_port(index) &= (uint8_t)~relay_bit(index);
#endif
}

void relay_off(uint8_t index)
{
    if (index >= RELAY_COUNT_INTERNAL)
    {
        return;
    }

#if RELAY_ACTIVE_HIGH
    *relay_port(index) &= (uint8_t)~relay_bit(index);
#else
    *relay_port(index) |= relay_bit(index);
#endif
}

bool relay_is_on(uint8_t index)
{
    bool level;

    if (index >= RELAY_COUNT_INTERNAL)
    {
        return false;
    }

    level = (*relay_port(index) & relay_bit(index)) != 0;

#if RELAY_ACTIVE_HIGH
    return level;
#else
    return !level;
#endif
}

uint8_t relay_mask(void)
{
    uint8_t i;
    uint8_t mask = 0;

    for (i = 0; i < RELAY_COUNT_INTERNAL; i++)
    {
        if (relay_is_on(i))
        {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
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
