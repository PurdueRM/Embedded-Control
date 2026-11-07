#include "stepper_motor_PWM.h"
#include "stm32f4xx_hal.h"


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "user_math.h"
#include <math.h>



extern TIM_HandleTypeDef htim; // Declare timer handle

Stepper_Motor_Handle_t *stepper_motor = NULL;


Stepper_Motor_Handle_t *Stepper_Motor_Init(int8_t pin) {
    Stepper_Motor_Handle_t *motor_handle = malloc(sizeof(Stepper_Motor_Handle_t));
    if (motor_handle == NULL) {
        return NULL;
    }

    motor_handle->pin = pin;

    motor_handle->direction = 0; // default to clockwise

    motor_handle->microstep_velocity = 0; // default to 0 microsteps per second
    motor_handle->velocity = 0; // default to 0 rpm

    HAL_TIM_PWM_Start(&htim, pin);
    motor_handle->PWM_enabled = true;
    motor_handle->PWM_duty_cycle_percent = 0; // default to 0 duty cycle
    
    
    return motor_handle;
}

void Stepper_Motor_Set_Velocity(Stepper_Motor_Handle_t *motor_handle, float velocity) {
    if (motor_handle == NULL) {
        return;
    }
    if (velocity * VELOCITY_TO_PWM > 1) {
        velocity = 1 / VELOCITY_TO_PWM;
    }
    motor_handle->velocity = velocity;
    motor_handle->PWM_duty_cycle_percent = (velocity * VELOCITY_TO_PWM); // scale velocity to PWM duty cycle
    __HAL_TIM_SET_COMPARE(&htim, motor_handle->pin, motor_handle->PWM_duty_cycle_percent * PWM_ARR);
}
