#include "debug_task.h"

#include "bsp_serial.h"
#include "remote.h"
#include "user_math.h"
#include "imu_task.h"
#include "robot.h"
#include "referee_system.h"
#include "jetson_orin.h"
#include "bsp_daemon.h"
#include "launch_task.h"
#include "two_bar_leg.h"
#include "dm_motor.h"
#include "wheel_legged_3d_lqr.h"
#include "dji_motor.h"

extern WheelLeggedInput g_wheel_legged_input;

extern WheelLeggedState g_wheel_legged_state;

extern DM_Motor_Handle_t *g_chassis_joint_motor[4];;
extern DJI_Motor_Handle_t *g_chassis_drive_motor_left;
extern DJI_Motor_Handle_t *g_chassis_drive_motor_right;
extern Two_Bar_Kinematics_t g_right_leg_kinematics;;

extern Two_Bar_Virtual_Force right_virtual_force;
extern Two_Bar_Motor_Torque motor_torque;

extern Robot_State_t g_robot_state;
extern IMU_t g_imu;
extern Remote_t g_remote;
extern Daemon_Instance_t *g_daemon_instances[3];
extern Daemon_Instance_t *g_remote_daemon;
extern Daemon_Instance_t *g_referee_daemon_instance_ptr;
extern float test_tmd;
// #define PRINT_RUNTIME_STATS
#ifdef PRINT_RUNTIME_STATS
char g_debug_buffer[1024 * 2] = {0};
#endif

const char *top_border = "\r\n\r\n\r\n/***** System Info *****/\r\n";
const char *bottom_border = "/***** End of Info *****/\r\n";

#define DEBUG_ENABLED
extern float g_chassis_right_angle_error;
extern float g_chassis_right_angle_error_original;
extern Two_Bar_Virtual_Force left_virtual_force;
extern Two_Bar_Virtual_Force right_virtual_force;
extern float g_chassis_left_drive_torque;
extern float g_chassis_right_drive_torque;
void Debug_Task_Loop(void)
{
#ifdef DEBUG_ENABLED
// static uint32_t counter = 0;
#ifdef PRINT_RUNTIME_STATS
    if (counter % 100 == 0) // Print every 100 cycles
    {
        vTaskGetRunTimeStats(g_debug_buffer);
        DEBUG_PRINTF(&huart6, "%s", top_border);
        DEBUG_PRINTF(&huart6, "%s", g_debug_buffer);
        DEBUG_PRINTF(&huart6, "%s", bottom_border);
    }
#endif

    // DEBUG_PRINTF(&huart1, ">theta:%.4f\n>leg:%.4f\n", g_right_leg_kinematics.theta, g_right_leg_kinematics.leg_length);
    // DEBUG_PRINTF(&huart1, ">supportive_force:%.4f\n>torque:%.4f\n", right_virtual_force.supportive_force, right_virtual_force.torque);
    // DEBUG_PRINTF(&huart1, ">error:%f\n", g_chassis_right_angle_error);
    // DEBUG_PRINTF(&huart1, ">original:%f\n", g_chassis_right_angle_error_original);
    // DEBUG_PRINTF(&huart1, ">motor0:%f\n>motor1:%f\n>motor2:%f\n>motor3:%f\n",
    //              g_chassis_joint_motor[0]->stats->pos, g_chassis_joint_motor[1]->stats->pos,
    //              g_chassis_joint_motor[2]->stats->pos, g_chassis_joint_motor[3]->stats->pos);
    // state vector one by one
    // DEBUG_PRINTF(&huart1, ">s:%.4f\n>ds:%.4f\n>phi:%.4f\n>dphi:%.4f\n>theta_ll:%.4f\n>dtheta_ll:%.4f\n>theta_lr:%.4f\n>dtheta_lr:%.4f\n>theta_b:%.4f\n>dtheta_b:%.4f\n",
    //              g_wheel_legged_state.s, g_wheel_legged_state.ds, g_wheel_legged_state.phi, g_wheel_legged_state.dphi,
    //              g_wheel_legged_state.theta_ll, g_wheel_legged_state.dtheta_ll,
    //              g_wheel_legged_state.theta_lr, g_wheel_legged_state.dtheta_lr,
    //              g_wheel_legged_state.theta_b, g_wheel_legged_state.dtheta_b);
    DEBUG_PRINTF(&huart1, ">s:%.4f\n", g_wheel_legged_state.s);
    DEBUG_PRINTF(&huart1, ">ds:%.4f\n", g_wheel_legged_state.ds);
    DEBUG_PRINTF(&huart1, ">phi:%.4f\n", g_wheel_legged_state.phi);
    DEBUG_PRINTF(&huart1, ">dphi:%.4f\n", g_wheel_legged_state.dphi);
    DEBUG_PRINTF(&huart1, ">theta_ll:%.4f\n", g_wheel_legged_state.theta_ll);
    DEBUG_PRINTF(&huart1, ">dtheta_ll:%.4f\n", g_wheel_legged_state.dtheta_ll);
    DEBUG_PRINTF(&huart1, ">theta_lr:%.4f\n", g_wheel_legged_state.theta_lr);
    DEBUG_PRINTF(&huart1, ">dtheta_lr:%.4f\n", g_wheel_legged_state.dtheta_lr);
    DEBUG_PRINTF(&huart1, ">theta_b:%.4f\n", g_wheel_legged_state.theta_b);
    DEBUG_PRINTF(&huart1, ">dtheta_b:%.4f\n", g_wheel_legged_state.dtheta_b);
    // DEBUG_PRINTF(&huart1, ">mtrpos1:%.4f\n", DJI_Motor_Get_Total_Angle(g_chassis_drive_motor_left));
    DEBUG_PRINTF(&huart1, ">T1:%.4f\n", left_virtual_force.torque);
    DEBUG_PRINTF(&huart1, ">T2:%.4f\n", right_virtual_force.torque);
    DEBUG_PRINTF(&huart1, ">mtr1:%f\n", g_chassis_left_drive_torque);
    DEBUG_PRINTF(&huart1, ">mtr2:%f\n", g_chassis_right_drive_torque);
    //  DEBUG_PRINTF(&huart6, ">time:%.1f\n>yaw:%f\n>pitch:%f\n>roll:%f\n", (float) counter / 1000.0f * DEBUG_PERIOD,
    //              g_imu.deg.yaw, g_imu.deg.pitch, g_imu.deg.roll);
    //  DEBUG_PRINTF(&huart6, ">remote_daemon:%d\n", g_remote_daemon->counter);
    //  counter++;
    //  if (counter > 0xFFFFFFFF) {
    //      counter = 0;
    //  }
#endif
}