/**
 * @file       imu_task.c/h
 * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
 *             enable data ready pin to save cpu time.enalbe bmi088 data ready
 *             enable spi DMA to save the time spi transmit
 */

#ifndef imu_task_H
#define imu_task_H
#include <stdint.h>
#include "main.h"
#include "cmsis_os.h"
#include "bsp_pwm.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "pid.h"
#include "MahonyAHRS.h"
#include "math.h"

#define RAD_TO_DEG 180.0f / 3.14159f

typedef struct Euler_Orientation
{
  float yaw;
  float pitch;
  float roll;
} Euler_Orientation_t;

typedef struct Quaternion_Orientation
{
  float yaw;
  float pitch;
  float roll;
} Quaternion_Orientation_t;

typedef struct IMU
{
  BMI088_Raw_t bmi088_raw;

  IST8310_Raw_t ist8310_raw;

  float quat[4];

  Euler_Orientation_t rad;

  Euler_Orientation_t deg;

} IMU_t;

extern IMU_t g_imu;

#endif
