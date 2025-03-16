/**
 * @file       imu_task.c/h
 * @brief      use bmi088 to calculate the euler angle.
 *
 * @note        change sample frequency in fusion
 */

#include "imu_task.h"



void IMU_Task_Init(IMU_t *imu);
void IMU_Task_Process(IMU_t *imu);
void IMU_Task_Temp();

IMU_t g_imu;
PID_t g_imu_temp_pid;

void IMU_Task(void const *pvParameters)
{
    IMU_Task_Init(&g_imu);
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);
    while (1)
    {
        IMU_Task_Process(&g_imu);
        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

void IMU_Task_Init(IMU_t *imu)
{
    PID_Init(&g_imu_temp_pid, 1600.0f, 0.2f, 0, 4500, 4400, 0);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    int error = BMI088_init();
    while (error)
    {
        error = BMI088_init();
        osDelay(100);
    }

    /* Initializing Quaternion */
    imu->quat[0] = 1.0f;
    imu->quat[1] = 0.0f;
    imu->quat[2] = 0.0f;
    imu->quat[3] = 0.0f;

}

void IMU_Task_Process(IMU_t *imu)
{
    BMI088_read(imu->bmi088_raw.gyro, imu->bmi088_raw.accel, &imu->bmi088_raw.temp);

    #ifdef WITH_MAGNETOMETER
    // fusion using magnetometer
     MahonyAHRSupdate(imu->quat,
                      imu->bmi088_raw.gyro[0], imu->bmi088_raw.gyro[1], imu->bmi088_raw.gyro[2],
                      imu->bmi088_raw.accel[0], imu->bmi088_raw.accel[1], imu->bmi088_raw.accel[2],
                      imu->ist8310_raw.mag[0], imu->ist8310_raw.mag[1], imu->ist8310_raw.mag[2]);
    #else
    // fusion without magnetometer
    MahonyAHRSupdateIMU(imu->quat,
                        imu->bmi088_raw.gyro[0], imu->bmi088_raw.gyro[1], imu->bmi088_raw.gyro[2],
                        imu->bmi088_raw.accel[0], imu->bmi088_raw.accel[1], imu->bmi088_raw.accel[2]);
    #endif
    /* Quternion to Euler */
    imu->rad.yaw = atan2f(2.0f * (imu->quat[0] * imu->quat[3] + imu->quat[1] * imu->quat[2]), 2.0f * (imu->quat[0] * imu->quat[0] + imu->quat[1] * imu->quat[1]) - 1.0f);
    imu->rad.pitch = asinf(-2.0f * (imu->quat[1] * imu->quat[3] - imu->quat[0] * imu->quat[2]));
    imu->rad.roll = atan2f(2.0f * (imu->quat[0] * imu->quat[1] + imu->quat[2] * imu->quat[3]), 2.0f * (imu->quat[0] * imu->quat[0] + imu->quat[3] * imu->quat[3]) - 1.0f);
    /* Radian to Degree*/
    imu->deg.yaw = imu->rad.yaw * RAD_TO_DEG;
    imu->deg.pitch = imu->rad.pitch * RAD_TO_DEG;
    imu->deg.roll = imu->rad.roll * RAD_TO_DEG;

    IMU_Task_Temp();
}

void IMU_Task_Temp() {
    static uint8_t start_complete = 0;
    if (g_imu.bmi088_raw.temp > 40.0f) {start_complete = 1;}
    switch (start_complete)
    {
    case 1:
    {
        uint16_t temp_pwm = (uint16_t) PID(&g_imu_temp_pid, 40 - g_imu.bmi088_raw.temp);
        __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, temp_pwm);
        break;
    }
    case 0:
        __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, 4999);
        break;

    default:
        break;
    }
}