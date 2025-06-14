#ifndef TWO_BAR_LEG_H
#define TWO_BAR_LEG_H

typedef struct {
    float leg_length;
    float theta;
    float theta1;
    float theta2;
} Two_Bar_Kinematics_t;

typedef struct {
    float supportive_force;
    float torque;
} Two_Bar_Virtual_Force;

typedef struct {
    float torque1;
    float torque2;
} Two_Bar_Motor_Torque;

typedef struct {
    float thigh_length;
    float calf_length;
} Two_Bar_Config_t;

extern void Two_Bar_Init(float thigh_length, float calf_length);
extern void Two_Bar_Forward_Kinematics(Two_Bar_Kinematics_t *kinematics, float theta1, float theta2);
extern void Two_Bar_Get_Motor_Torque_From_Virtual_Force(Two_Bar_Kinematics_t *stats, Two_Bar_Virtual_Force *virtual_force, Two_Bar_Motor_Torque *motor_torque);

#endif // TWO_BAR_LEG_H