#ifndef F_CPU
#warning "F_CPU is not defined. Defaulting to 1000000UL"
#define F_CPU 1000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "timer.h"

static volatile uint32_t g_millis = 0;

/*
 * Timer1 CTC calculation:
 *
 * Timer frequency = F_CPU / prescaler
 * Required interrupt frequency = 1000 Hz
 *
 * We use prescaler = 1.
 *
 * OCR1A = F_CPU / 1000 - 1
 *
 * The formula below rounds the value if F_CPU is not exactly divisible by 1000.
 */
#define TIMER1_PRESCALER       1UL
#define TIMER1_COMPARE_VALUE   (((F_CPU / TIMER1_PRESCALER) + 500UL) / 1000UL - 1UL)

void timer1_init_1ms(void)
{
    cli();

    /*
     * Stop Timer1 before configuration
     */
    TCCR1A = 0x00;
    TCCR1B = 0x00;

    /*
     * Clear counter
     */
    TCNT1 = 0;

    /*
     * Set compare value for 1 ms
     */
    OCR1A = (uint16_t)TIMER1_COMPARE_VALUE;

    /*
     * Enable Timer1 Compare A interrupt
     */
    TIMSK |= (1 << OCIE1A);

    /*
     * CTC mode:
     * WGM12 = 1
     *
     * Clock source:
     * CS10 = 1 ? prescaler = 1
     */
    TCCR1B = (1 << WGM12) | (1 << CS10);

    sei();
}

ISR(TIMER1_COMPA_vect)
{
    g_millis++;
}

uint32_t millis(void)
{
    uint32_t value;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        value = g_millis;
    }

    return value;
}