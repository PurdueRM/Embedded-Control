#ifndef LAUNCH_TASK_H
#define LAUNCH_TASK_H

#include <stdint.h>
#include "dji_motor.h"

#define FLYWHEEL_VELOCITY_LOW_FOR_DEBUG (1000.0f * M3508_REDUCTION_RATIO)
#define FLYWHEEL_VELOCITY_10 (3000.0f * M3508_REDUCTION_RATIO)
#define FLYWHEEL_VELOCITY_20 (5000.0f * M3508_REDUCTION_RATIO)
#define FLYWHEEL_VELOCITY_30 (7000.0f * M3508_REDUCTION_RATIO)
#define FEED_HOLE_NUM (6.0f)
#define LAUNCH_FREQUENCY (15.0f)
#define LAUNCH_PERIOD (1000.0f/LAUNCH_FREQUENCY)
#define FEED_1_PROJECTILE_ANGLE (2.0f*PI/FEED_HOLE_NUM)
#define FEED_FREQUENCY_6 (6.0f / FEED_HOLE_NUM * 60.0f)
#define FEED_FREQUENCY_12 (12.0f / FEED_HOLE_NUM * 60.0f)
#define FEED_FREQUENCY_15 (15.0f / FEED_HOLE_NUM * 60.0f)
#define FEED_FREQUENCY_20 (20.0f / FEED_HOLE_NUM * 60.0f)
#define FEED_FREQUENCY_30 (30.0f / FEED_HOLE_NUM * 60.0f)

//#define HEAT_CONTROL

typedef struct
{
    float flywheel_velocity;
    float feed_angle;
    float feed_velocity;
    uint8_t single_launch_flag;
    uint8_t single_launch_finished_flag;
    uint8_t burst_launch_flag;
    uint8_t prev_burst_launch_flag;
    uint8_t flywheel_enabled;
    uint8_t reverse_burst_launch_pending_flag;

    int16_t calculated_heat;
    uint16_t heat_count;
    uint16_t launch_freq_count;
    uint8_t reverse_flag;
    uint8_t prev_reverse_flag;
} Launch_Target_t;

void Launch_Task_Init(void);
void Launch_Ctrl_Loop(void);
void Servo_Init(void);
void Servo_SetAngle(float);
void Disable_Servo(void);


extern Launch_Target_t g_launch_target;
extern DJI_Motor_Handle_t *g_flywheel_left, *g_flywheel_right, *g_motor_feed;

#endif // LAUNCH_TASK_H