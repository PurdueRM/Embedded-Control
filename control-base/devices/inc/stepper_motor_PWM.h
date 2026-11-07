#ifndef STEPPER_MOTOR_PWM_H
    #define STEPPER_MOTOR_PWM_H

#include <stdint.h>
#include <stdbool.h>

#define PWM_ARR 1000 // Define the PWM auto-reload register value
#define VELOCITY_TO_PWM 0.1 //ABSOLUTLY MADE UP PLEASE CHANGE


typedef struct DDSM_Motor {

    int8_t pin;

    bool direction; // 0 for clockwise, 1 for counterclockwise
    

    float velocity; // rpm
    int32_t microstep_velocity; // microsteps per second


    bool PWM_enabled;
    int32_t PWM_duty_cycle_percent; // duty cycle in percentage


} Stepper_Motor_Handle_t;

Stepper_Motor_Handle_t* Stepper_Motor_Init(int8_t pin);
void Stepper_Motor_Set_Velocity(Stepper_Motor_Handle_t *motor_handle, float velocity);

#endif

