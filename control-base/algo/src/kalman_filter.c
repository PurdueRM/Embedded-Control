#include "kalman_filter.h"

float First_Order_Kalman_Filter(Kalman_Filter_t *KF, float Measurement)
{
	KF->Current_P = KF->Prev_P + KF->Q;
	KF->K = KF->Current_P / (KF->Current_P + KF->R);
	KF->Output = KF->Output + KF->K*(Measurement - KF->Output);
	KF->Prev_P = (1 - KF->K)*KF->Current_P;
	
	return KF->Output;
}