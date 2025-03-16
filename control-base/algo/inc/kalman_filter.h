#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include <stdio.h>
#include <stdint.h>

typedef struct
{
	float Prev_P;
	float Current_P;
	float Output;
	float K;
	float Q;
	float R;
}Kalman_Filter_t;

float First_Order_Kalman_Filter(Kalman_Filter_t *KF, float Measurement);

#endif // __KALMAN_FILTER_H