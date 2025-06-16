#ifndef WHEEL_LEGGED_3D_LQR_H
#define WHEEL_LEGGED_3D_LQR_H
typedef struct {
    float s;
    float ds;
    float phi;
    float dphi;
    float theta_ll;
    float dtheta_ll;
    float theta_lr;
    float dtheta_lr;
    float theta_b;
    float dtheta_b;

    // for calculation
    float s_prev;
    float s_offset;
    float phi_prev;
    float theta_ll_prev;
    float theta_lr_prev;
    float theta_b_prev;
} WheelLeggedState;

typedef struct {
    float T_wl;
    float T_wr;
    float T_bl;
    float T_br;
} WheelLeggedInput;

void Wheel_Legged_Compute_LQR_output(const WheelLeggedState *state, float l_l, float l_r, WheelLeggedInput *input);

#endif // WHEEL_LEGGED_3D_LQR_H