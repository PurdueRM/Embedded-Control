#ifndef ROTOR_H
#define ROTOR_H
#include <stdint.h>

typedef struct PWM_Pulse
{
  uint16_t pwm1;
  uint16_t pwm2;
  uint16_t pwm3;
  uint16_t pwm4;
  uint16_t pwm5;
  uint16_t pwm6;
  uint16_t pwm7;

} PWM_Pulse_t;

extern PWM_Pulse_t PWM_Pulse;
extern void pwmSetCompare(PWM_Pulse_t PWM_Pulse);

#endif
