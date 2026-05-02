#include "launch_task.h"

#include "dji_motor.h"
#include "motor.h"
#include "robot.h"
#include "remote.h"
#include "user_math.h"
#include "referee_system.h"
#include "laser.h"
#include <stdint.h>

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
Launch_Target_t g_launch_target;
DJI_Motor_Handle_t *g_flywheel_left, *g_flywheel_right, *g_feed_motor;

void Launch_Task_Init()
{
    
}

void Launch_Ctrl_Loop()
{
    
    
}