#include "launch_task.h"

#include "dji_motor.h"
#include "motor.h"
#include "robot.h"
#include "remote.h"
#include "user_math.h"
#include "referee_system.h"
#include "laser.h"
#include <stdint.h>
#include "jetson_orin.h"
#include "bsp_pwm.h"

extern TIM_HandleTypeDef htim1;   // change this if your PWM uses another timer

PWM_Instance_t *servo_pwm;

// TODO: Copied from Swerve-Standard

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

DJI_Motor_Handle_t *g_flywheel_left, *g_flywheel_right, *g_feed_motor;

void Launch_Task_Init()
{
    Servo_Init();
}

void Launch_Ctrl_Loop()
{
     static uint32_t last_time = 0;
    // static uint8_t state = 0;

    if (HAL_GetTick() - last_time < 1000)
        return;

    last_time = HAL_GetTick();

    // if (state == 0) {
    //     Servo_SetAngle(0);
    //     state = 1;
    // } else if (state == 1) {
    //     Servo_SetAngle(90);
    //     state = 2;
    // } else {
    //     Servo_SetAngle(180);
    //     state = 0;
    // }
    Servo_SetAngle(0);
}



void Servo_Init(void)
{
    PWM_Config_t servo_config = {
        .htim = &htim1,
        .channel = TIM_CHANNEL_1,   // change to your actual PWM channel
        .period = 0.020f,           // 20 ms period = 50 Hz
        .dutyratio = 0.075f,        // center position
        .id = 0
    };

    servo_pwm = PWM_Register(&servo_config);
}
void Servo_SetAngle(float angle)
{
    if (angle < 0.0f)
        angle = 0.0f;
    if (angle > 180.0f)
        angle = 180.0f;

    float pulse_ms = 1.0f + angle * (1.0f / 180.0f);

    float duty = pulse_ms / 20.0f;

    PWM_Set_Duty_Ratio(servo_pwm, duty);
}