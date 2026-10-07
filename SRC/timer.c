#ifndef F_CPU
#error "F_CPU is not defined (expected -DF_CPU=4000000UL from the Makefile)"
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "timer.h"

#if F_CPU != 4000000UL
#error "timer2_init_1ms() is calculated for F_CPU = 4 MHz"
#endif

static volatile uint32_t g_millis = 0;

/*
 * Does not touch the global interrupt flag: the caller enables
 * interrupts with sei() once all peripherals are initialized.
 */
void timer2_init_1ms(void)
{
    /*
     * Timer2 CTC mode
     * F_CPU = 4 MHz, prescaler = 32 -> 125 kHz
     * OCR2 = 124 -> 125 counts -> exactly 1 ms
     */
    TCCR2 = 0x00;
    TCNT2 = 0;
    OCR2  = 124;

    /* WGM21 = 1 -> CTC, CS21 = 1, CS20 = 1 -> prescaler 32 */
    TCCR2 = (1 << WGM21) | (1 << CS21) | (1 << CS20);

    TIMSK |= (1 << OCIE2);
}

ISR(TIMER2_COMP_vect)
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
