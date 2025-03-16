
#include "laser.h"
#include "tim.h"

void Laser_Init()
{
    // HAL_TIM_Base_Start(&htim3); // This is initialized in tim.h
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void Laser_On()
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 65535/250);
}
void Laser_Off()
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
}