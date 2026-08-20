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
PWM_Instance_t *pump_pwm;
PWM_Instance_t *valve_pwm;

// TODO: Copied from Swerve-Standard

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

DJI_Motor_Handle_t *g_flywheel_left, *g_flywheel_right, *g_feed_motor;
float servo_angle = 0;

void Launch_Task_Init()
{
    Servo_Init();
    // Disable_Servo();

    Valve_Init();
    Pump_Init();

    Valve_TurnOff();
    Pump_TurnOff();
    
    servo_angle = 0;
}

void Launch_Ctrl_Loop()
{
    //  static uint32_t last_time = 0;
    // static uint8_t state = 0;

    // if (HAL_GetTick() - last_time < 1000)
    //     return;

    // last_time = HAL_GetTick();

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
    if(g_remote.controller.left_switch == 1){
        Valve_TurnOff();
        Pump_TurnOn();

    }
    else {
        Valve_TurnOn();
        // Pump_TurnOff();
    }
    // Servo_SetAngle(servo_angle);

    // Disable_Servo();
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

void Valve_Init(void) {
    PWM_Config_t valve_config = {
        .htim = &htim1,
        .channel = TIM_CHANNEL_2,
        .period = 0.020f, // 20 ms period
        .dutyratio = 0.05f, // closed
        .id = 1
    };

    valve_pwm = PWM_Register(&valve_config);
}

void Pump_Init(void) {
    PWM_Config_t pump_config = {
        .htim = &htim1,
        .channel = TIM_CHANNEL_3,
        .period = 0.020f, // 20 ms period
        .dutyratio = 0.05f, // off
        .id = 2
    };

    pump_pwm = PWM_Register(&pump_config);

}

void Valve_TurnOn() {
    float duty = 0.10f; // 2000 us pulse / 20000 us period = 0.10 (10%)
    PWM_Set_Duty_Ratio(valve_pwm, duty);
}

void Valve_TurnOff() {
    float duty = 0.05f; // 1000 us pulse / 20000 us period = 0.05 (5%)
    PWM_Set_Duty_Ratio(valve_pwm, duty);
}

void Pump_TurnOn() {
    // 2000 us pulse / 20000 us period = 0.10 (10%)
    float duty = 0.10f; 
    PWM_Set_Duty_Ratio(pump_pwm, duty);
}

void Pump_TurnOff() {
    // 1000 us pulse / 20000 us period = 0.05 (5%)
    float duty = 0.05f; 
    PWM_Set_Duty_Ratio(pump_pwm, duty);
}

void Servo_SetAngle(float angle)
{
    if (angle < 0.0f)
        angle = 0.0f;
    if (angle > 80.0f)
        angle = 80.0f;

    // float pulse_ms = 1.0f + angle * (1.0f / 180.0f); // 1 ms for 0 degrees, 2 ms for 180 degrees
     float pulse_ms = 0.5f + angle * (2.0f / 180.0f); // 0.5 ms for 0 degrees, 2.5 ms for 180 degrees

    float duty = pulse_ms / 20.0f;

    PWM_Set_Duty_Ratio(servo_pwm, duty);
}

void Disable_Servo(){
    PWM_Set_Duty_Ratio(servo_pwm, 0);
}