#include "PWM.h"
//init registers

void pwm_init(void)
{
  TCCR1A = (1<<WGM11) | (1<<WGM10) | (1<<COM1A1); // 10bits resolution, 0 when count up
  TCCR1B = (0<<CS12) | (0<<CS11) | (1<<CS10);     // prescaler CK
  TCNT1 =0;                                       // Start count from 0
  OCR1A = 0xFF;                                   // initial PWM value

  ACSR = (1<<ACD);
}
