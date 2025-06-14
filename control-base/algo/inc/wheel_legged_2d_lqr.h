#ifndef WHEEL_LEGGED_2D_LQR_H
#define WHEEL_LEGGED_2D_LQR_H

typedef struct lqr_ss_s {
    float target_x;
    float target_x_dot;

    float x;
    float x_dot;
    float theta;
    float theta_dot;
    float phi;
    float phi_dot;
    float leg_len;
} lqr_ss_t;

typedef struct lqr_u_s {
    float T_A;
    float T_B;
} lqr_u_t;


// #define LQR11 (0.6982f)             //#define LQR11 (0.9975f)
// #define LQR12 (1.5555f)             //#define LQR12 (2.4332f)
// #define LQR13 (-10.2536f)           //#define LQR13 (-18.3302f)
// #define LQR14 (-1.2820f)            //#define LQR14 (-2.2439f)
// #define LQR15 (-2.3611f)            //#define LQR15 (-4.2870f)
// #define LQR16 (-0.9715f)            //#define LQR16 (-0.6388f)
// #define LQR21 (-1.6008f)            //#define LQR21 (-0.1583f)
// #define LQR22 (-3.2712f)            //#define LQR22 (-0.3134f)
// #define LQR23 (10.5222f)            //#define LQR23 (4.5931f)
// #define LQR24 (1.3224f)             //#define LQR24 (0.1308f)
// #define LQR25 (-2.0104f)            //#define LQR25 (-8.2535f)
// #define LQR26 (-1.1699f)            //#define LQR26 (-1.8777f)

//   -56.5101   -8.1049  -19.4735  -24.9331   25.0460    3.7710
//    75.0944   10.4548   36.1097   42.0896   42.2742    0.6126

//   -18.5278   -2.2124   -3.8514   -5.2588    6.9478    1.2402
//    34.5401    4.5301   13.1021   16.2025   12.0309    0.5259

// #define LQR11 (-18.5278f)
// #define LQR12 (-2.2124f)
// #define LQR13 (-3.8514f)
// #define LQR14 (-5.2588f)
// #define LQR15 (6.9478f)
// #define LQR16 (1.2402f)
// #define LQR21 (34.5401f)
// #define LQR22 (4.5301f)
// #define LQR23 (13.1021f)
// #define LQR24 (16.2025f)
// #define LQR25 (12.0309f)
// #define LQR26 (0.5259f)


// #define LQR11 (-44.3788f)
// #define LQR12 (-6.8496f)
// #define LQR13 (-22.2828f)
// #define LQR14 (-21.5569f)
// #define LQR15 (28.7706f)
// #define LQR16 (4.3751f)

// #define LQR21 (11.2006f)
// #define LQR22 (0.7339f)
// #define LQR23 (3.7300f)
// #define LQR24 (3.2058f)
// #define LQR25 (151.7300f)
// #define LQR26 (4.6387f)

// #define LQR11 (-22.8654f)
// #define LQR12 (-2.7250f)
// #define LQR13 (-19.2061f)
// #define LQR14 (-15.1164f)
// #define LQR15 (41.4567f)
// #define LQR16 (3.8236f)
// #define LQR21 (19.5479f)
// #define LQR22 (2.6123f)
// #define LQR23 (22.9022f)
// #define LQR24 (16.8579f)
// #define LQR25 (119.4593f)
// #define LQR26 (3.9394f)

//    -5.0925   -0.3268   -0.0117   -0.0839    4.0051    0.6872
//    26.0664    2.1179    4.4718    8.7205   -3.3557   -1.3210



// #define LQR11 (-56.5101f)
// #define LQR12 (-8.1049f)
// #define LQR13 (-19.4735f)
// #define LQR14 (-24.9331f)
// #define LQR15 (25.0460f)
// #define LQR16 (3.7710f)
// #define LQR21 (75.0944f)
// #define LQR22 (10.4548f)
// #define LQR23 (36.1097f)
// #define LQR24 (42.0896f)
// #define LQR25 (42.2742f)
// #define LQR26 (0.6126f)


// #define LQR11 (-129.0240f)
// #define LQR12 (-19.1624f)
// #define LQR13 (-50.1387f)
// #define LQR14 (-62.9774f)
// #define LQR15 (45.1771f)
// #define LQR16 (7.7063f)
// #define LQR21 (50.1380f)
// #define LQR22 (6.6594f)
// #define LQR23 (26.1938f)
// #define LQR24 (29.5130f)
// #define LQR25 (52.7310f)
// #define LQR26 (2.1777f)


void LQR_Output(lqr_u_t *u, lqr_ss_t *state);

#endif // WHEEL_LEGGED_2D_LQR_H