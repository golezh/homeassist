#ifndef PWM_H
#define PWM_H

#include <stdint.h>
#include <avr/io.h>

/* Timer1 10-bit phase-correct PWM on OC1A (PD5) */
void pwm_init(void);
void pwm_set(uint16_t value);   // 0..1023

#endif // PWM_H
