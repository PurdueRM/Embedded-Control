#include "wheel_legged_2d_Lqr.h"
#include <stdint.h>
#include "robot.h"
#define LQR11 (-9.0381f)
#define LQR12 (-0.8002f)
#define LQR13 (-1.5454f)
#define LQR14 (-1.8952f)
#define LQR15 (7.0606f)
#define LQR16 (0.8181f)
#define LQR21 (29.8781f)
#define LQR22 (3.7447f)
#define LQR23 (11.3292f)
#define LQR24 (12.9867f)
#define LQR25 (25.9207f)
#define LQR26 (0.8019f)

#define OPENSOURCE_COORDINATE
//    Q=diag([0.1 0.1 100 50 5000 1]);
//    R=diag([150 5]);  
// float lqr_poly_fit_param[12][4] = 
// {{-146.8397,178.1973,-83.7881,2.8443},
// {8.5304,-3.7974,-3.5802,0.2173},
// {-40.5217,39.2950,-13.5266,0.9230},
// {-57.9757,56.6803,-19.9446,1.3387},
// {195.0357,-122.9509,9.7593,6.2244},
// {28.3143,-20.0813,3.1549,0.5661},
// {1473.2038,-1142.9067,247.5109,5.8574},
// {105.5269,-94.7226,27.2942,-0.5018},
// {135.5466,-83.0411,4.9135,4.6654},
// {206.4120,-129.2789,9.6747,6.7037},
// {1603.1755,-1567.6944,545.5435,-38.7507},
// {130.3261,-137.4586,52.2075,-5.0431}};

// Q=diag([0.1 0.1 100 50 5000 1]);
// R=[150 0;0 5];
// float lqr_poly_fit_param[12][4] = 
// {{-135.3453,158.9821,-75.0086,2.0930},
// {4.8803,-1.4456,-3.8973,0.1909},
// {-37.5561,35.2304,-11.7066,0.6412},
// {-54.4328,51.3769,-17.4431,0.9291},
// {138.9470,-77.9373,-2.4892,7.9823},
// {21.5725,-14.9521,2.0154,0.6933},
// {1117.0967,-875.1741,193.5456,7.1032},
// {85.3890,-76.2583,21.8655,-0.1857},
// {65.3828,-28.1241,-8.2274,5.3604},
// {103.0676,-47.8553,-10.0767,7.8005},
// {1622.2835,-1555.2473,533.1584,-34.9396},
// {129.6584,-132.4556,49.3328,-4.6592}};


//    Q=diag([0.1 0.1 100 500 5000 1]);
//    R=[150 0;0 5];
// float lqr_poly_fit_param[12][4] = 
// {{-75.0118,112.1647,-70.4753,1.8061},
// {12.9513,-8.7708,-3.6872,0.1924},
// {-26.5521,26.3310,-9.4944,0.5264},
// {-68.3177,68.2261,-25.1308,1.3668},
// {130.5731,-82.6278,3.4079,7.5875},
// {18.2980,-13.4825,2.1852,0.6954},
// {1191.5062,-1006.2760,259.2660,5.5796},
// {96.2733,-95.5380,33.4083,-0.3565},
// {65.3363,-35.3815,-3.4411,4.9918},
// {179.6229,-101.0996,-6.1644,13.0344},
// {1158.9919,-1180.8042,442.3189,-31.6758},
// {93.3741,-102.2150,41.9548,-4.6102}};


//    Q=diag([0.1 0.1 100 1600 5000 1]);
//    R=[100 0;0 5];

extern Robot_State_t g_robot_state;
float lqr_poly_fit_param[12][4] = 
{{-40.4426,92.2805,-83.0307,1.9406},
{22.7498,-17.8524,-5.2327,0.2516},
{-25.5025,25.6231,-9.5258,0.4409},
{-106.6486,107.8056,-40.8232,1.8520},
{115.6430,-73.3241,0.9146,9.4114},
{16.9954,-12.9700,2.2799,0.8896},
{1188.7971,-1067.5053,309.3857,5.7028},
{104.6622,-112.2293,45.3611,-0.3419},
{42.0172,-20.5095,-5.3839,4.9969},
{187.4530,-96.2940,-19.6252,21.2350},
{942.3723,-975.0634,379.1022,-27.2181},
{80.8579,-89.0861,37.8765,-4.6313}};
void LQR_Output(lqr_u_t *u, lqr_ss_t *state)
{
float len = state->leg_len;
float len_sqrt = len * len;
float len_cub = len * len * len;
// if (g_robot_state.spintop_mode){
//     u->T_A = (lqr_poly_fit_param[0 ][0] * len_cub + lqr_poly_fit_param[0 ][1] * len_sqrt + lqr_poly_fit_param[0 ][2] * len + lqr_poly_fit_param[0 ][3]) * -state->theta +
//              (lqr_poly_fit_param[1 ][0] * len_cub + lqr_poly_fit_param[1 ][1] * len_sqrt + lqr_poly_fit_param[1 ][2] * len + lqr_poly_fit_param[1 ][3]) * -state->theta_dot +
//              (lqr_poly_fit_param[2 ][0] * len_cub + lqr_poly_fit_param[2 ][1] * len_sqrt + lqr_poly_fit_param[2 ][2] * len + lqr_poly_fit_param[2 ][3]) * (state->target_x-state->x) * 0 +
//              (lqr_poly_fit_param[3 ][0] * len_cub + lqr_poly_fit_param[3 ][1] * len_sqrt + lqr_poly_fit_param[3 ][2] * len + lqr_poly_fit_param[3 ][3]) * (state->target_x_dot - state->x_dot) +
//              (lqr_poly_fit_param[4 ][0] * len_cub + lqr_poly_fit_param[4 ][1] * len_sqrt + lqr_poly_fit_param[4 ][2] * len + lqr_poly_fit_param[4 ][3]) * (-state->phi) +
//              (lqr_poly_fit_param[5 ][0] * len_cub + lqr_poly_fit_param[5 ][1] * len_sqrt + lqr_poly_fit_param[5 ][2] * len + lqr_poly_fit_param[5 ][3]) * -state->phi_dot;
//     u->T_B = (lqr_poly_fit_param[6 ][0] * len_cub + lqr_poly_fit_param[6 ][1] * len_sqrt + lqr_poly_fit_param[6 ][2] * len + lqr_poly_fit_param[6 ][3]) * -state->theta +
//              (lqr_poly_fit_param[7 ][0] * len_cub + lqr_poly_fit_param[7 ][1] * len_sqrt + lqr_poly_fit_param[7 ][2] * len + lqr_poly_fit_param[7 ][3]) * -state->theta_dot +
//              (lqr_poly_fit_param[8 ][0] * len_cub + lqr_poly_fit_param[8 ][1] * len_sqrt + lqr_poly_fit_param[8 ][2] * len + lqr_poly_fit_param[8 ][3]) * (state->target_x-state->x) * 0+
//              (lqr_poly_fit_param[9 ][0] * len_cub + lqr_poly_fit_param[9 ][1] * len_sqrt + lqr_poly_fit_param[9 ][2] * len + lqr_poly_fit_param[9 ][3]) * (state->target_x_dot-state->x_dot) +
//              (lqr_poly_fit_param[10][0] * len_cub + lqr_poly_fit_param[10][1] * len_sqrt + lqr_poly_fit_param[10][2] * len + lqr_poly_fit_param[10][3]) * (-state->phi) +
//              (lqr_poly_fit_param[11][0] * len_cub + lqr_poly_fit_param[11][1] * len_sqrt + lqr_poly_fit_param[11][2] * len + lqr_poly_fit_param[11][3]) * -state->phi_dot;

// }
// else
// {
    u->T_A = (lqr_poly_fit_param[0 ][0] * len_cub + lqr_poly_fit_param[0 ][1] * len_sqrt + lqr_poly_fit_param[0 ][2] * len + lqr_poly_fit_param[0 ][3]) * -state->theta +
             (lqr_poly_fit_param[1 ][0] * len_cub + lqr_poly_fit_param[1 ][1] * len_sqrt + lqr_poly_fit_param[1 ][2] * len + lqr_poly_fit_param[1 ][3]) * -state->theta_dot +
             (lqr_poly_fit_param[2 ][0] * len_cub + lqr_poly_fit_param[2 ][1] * len_sqrt + lqr_poly_fit_param[2 ][2] * len + lqr_poly_fit_param[2 ][3]) * (state->target_x-state->x) +
             (lqr_poly_fit_param[3 ][0] * len_cub + lqr_poly_fit_param[3 ][1] * len_sqrt + lqr_poly_fit_param[3 ][2] * len + lqr_poly_fit_param[3 ][3]) * (state->target_x_dot - state->x_dot) +
             (lqr_poly_fit_param[4 ][0] * len_cub + lqr_poly_fit_param[4 ][1] * len_sqrt + lqr_poly_fit_param[4 ][2] * len + lqr_poly_fit_param[4 ][3]) * (-state->phi) +
             (lqr_poly_fit_param[5 ][0] * len_cub + lqr_poly_fit_param[5 ][1] * len_sqrt + lqr_poly_fit_param[5 ][2] * len + lqr_poly_fit_param[5 ][3]) * -state->phi_dot;
    u->T_B = (lqr_poly_fit_param[6 ][0] * len_cub + lqr_poly_fit_param[6 ][1] * len_sqrt + lqr_poly_fit_param[6 ][2] * len + lqr_poly_fit_param[6 ][3]) * -state->theta +
             (lqr_poly_fit_param[7 ][0] * len_cub + lqr_poly_fit_param[7 ][1] * len_sqrt + lqr_poly_fit_param[7 ][2] * len + lqr_poly_fit_param[7 ][3]) * -state->theta_dot +
             (lqr_poly_fit_param[8 ][0] * len_cub + lqr_poly_fit_param[8 ][1] * len_sqrt + lqr_poly_fit_param[8 ][2] * len + lqr_poly_fit_param[8 ][3]) * (state->target_x-state->x) +
             (lqr_poly_fit_param[9 ][0] * len_cub + lqr_poly_fit_param[9 ][1] * len_sqrt + lqr_poly_fit_param[9 ][2] * len + lqr_poly_fit_param[9 ][3]) * (state->target_x_dot-state->x_dot) +
             (lqr_poly_fit_param[10][0] * len_cub + lqr_poly_fit_param[10][1] * len_sqrt + lqr_poly_fit_param[10][2] * len + lqr_poly_fit_param[10][3]) * (-state->phi) +
             (lqr_poly_fit_param[11][0] * len_cub + lqr_poly_fit_param[11][1] * len_sqrt + lqr_poly_fit_param[11][2] * len + lqr_poly_fit_param[11][3]) * -state->phi_dot;
// }
}