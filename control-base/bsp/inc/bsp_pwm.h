#ifndef BSP_IMU_PWM_H
#define BSP_IMU_PWM_H
#include "tim.h"
#include <stdint.h>

typedef struct pwm_ins_temp
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t tclk;
    float period;
    float dutyratio;
    uint8_t id;
} PWM_Instance_t;

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float period;
    float dutyratio;
    uint8_t id;
} PWM_Config_t;

PWM_Instance_t *PWM_Register(PWM_Config_t *config);
void PWM_Start(PWM_Instance_t *pwm);
void PWM_Stop(PWM_Instance_t *pwm);
void PWM_Set_Period(PWM_Instance_t *pwm, float period);
void PWM_Set_Frequency(PWM_Instance_t *pwm, float freq);
void PWM_Set_Duty_Ratio(PWM_Instance_t *pwm, float dutyratio);

#endif
