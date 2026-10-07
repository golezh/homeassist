#include "PWM.h"
//init registers

void pwm_init(void)
{
    /*
     * OC1A = PD5
     */
    DDRD |= (1 << PD5);

    /*
     * Timer1 Phase Correct PWM, 10-bit
     * Non-inverting output on OC1A
     */
    TCCR1A = (1 << WGM11) | (1 << WGM10) | (1 << COM1A1);

    /*
     * No prescaler
     */
    TCCR1B = (1 << CS10);

    TCNT1 = 0;

    /*
     * Initial duty cycle about 25%
     * Range: 0..1023
     */
    OCR1A = 255;

    /*
     * Disable analog comparator
     */
    ACSR = (1 << ACD);
}

void pwm_set(uint16_t value)
{
    if (value > 1023)
    {
        value = 1023;
    }

    OCR1A = value;
}