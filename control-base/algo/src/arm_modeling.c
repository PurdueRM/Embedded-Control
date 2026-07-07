#include <stdio.h>
#include <math.h>

#include "user_math.h"
#include "arm_modeling.h"


// [a, alpha, d, theta]
Arm_State_t g_arm_state = { 
  .joints[0].dh_params = {0.0f, PI/2, H1, 0.0f},
  .joints[1].dh_params = {L1, 0.0f, 0.0f, PI/2},
  .joints[2].dh_params = {H2, PI/2, 0.0f, PI/2},
  .joints[3].dh_params = {0.0f, -1.0f * PI/2, L2 + H3, 0.0f},
  .joints[4].dh_params = {0.0f, -1.0f * PI/2, 0.0f, 0.0f},
  .joints[5].dh_params = {0.0f, 0.0f, H4, 0.0f},

  .sim_joints[0].dh_params = {0.0f, PI/2, H1, 0.0f},
  .sim_joints[1].dh_params = {L1, 0.0f, 0.0f, PI/2},
  .sim_joints[2].dh_params = {H2, PI/2, 0.0f, PI/2},
  .sim_joints[3].dh_params = {0.0f, -1.0f * PI/2, L2 + H3, 0.0f},
  .sim_joints[4].dh_params = {0.0f, -1.0f * PI/2, 0.0f, 0.0f},
  .sim_joints[5].dh_params = {0.0f, 0.0f, H4, 0.0f},
};

void init_arm_state(Arm_State_t *state) {
  state->jacobian            = new_mat(5, 6);
  state->pseudo_inv_jacobian = new_mat(6, 5);
  state->nullspace           = new_mat(6, 6);

  for (int i = 0; i < 7; i++) {
    state->transforms[i]     = new_eye(4);
    state->sim_transforms[i] = new_eye(4);
  }

  state->desired_orientation_mat = new_mat(3, 3);
  state->current_orientation_mat = new_mat(3, 3);
  state->orientation_error       = new_vec(3);

  state->wrist_pos = new_vec(3);

  state->theta = new_vec(6);
  state->theta_lims = new_mat(6, 2);

  state->buffer1_6x6 = new_mat(6, 6);
  state->buffer2_6x6 = new_mat(6, 6);

  state->buffer1_6x5 = new_mat(6, 5);

  state->buffer1_5x5 = new_mat(5, 5);
  state->buffer2_5x5 = new_mat(5, 5);
  state->buffer3_5x5 = new_mat(5, 5);

  state->buffer1_4x4 = new_mat(4, 4);
  state->buffer2_4x4 = new_mat(4, 4);

  state->buffer1_3x3 = new_mat(3, 3);
  state->buffer2_3x3 = new_mat(3, 3);
  state->buffer3_3x3 = new_mat(3, 3);

  state->buffer1_3x1 = new_vec(3);
  state->buffer2_3x1 = new_vec(3);

  state->buffer1_5x1 = new_vec(5);

  state->buffer1_6x1 = new_vec(6);
  state->buffer2_6x1 = new_vec(6);
  state->buffer3_6x1 = new_vec(6);

  MAT_IDX(g_arm_state.theta_lims, 0, 0) = -PI;
  MAT_IDX(g_arm_state.theta_lims, 0, 1) =  PI;

  MAT_IDX(g_arm_state.theta_lims, 1, 0) = -PI / 2;
  MAT_IDX(g_arm_state.theta_lims, 1, 1) =  PI / 2;

  MAT_IDX(g_arm_state.theta_lims, 2, 0) = -PI;
  MAT_IDX(g_arm_state.theta_lims, 2, 1) =  PI;

  MAT_IDX(g_arm_state.theta_lims, 3, 0) = -PI;
  MAT_IDX(g_arm_state.theta_lims, 3, 1) =  PI;

  MAT_IDX(g_arm_state.theta_lims, 4, 0) = -PI;
  MAT_IDX(g_arm_state.theta_lims, 4, 1) =  PI;

  MAT_IDX(g_arm_state.theta_lims, 5, 0) = -PI;
  MAT_IDX(g_arm_state.theta_lims, 5, 1) =  PI;
}

Mat* joint_transform(joint_t joint) {
  return dh_transform(joint.dh_params);
}

Mat* joint_transform_buffer(joint_t joint, Mat* buffer) {
  return dh_transform_buffer(joint.dh_params, buffer);
}

void dh_update_arm() {
  g_arm_state.joints[0].dh_params.theta = g_arm_state.joints[0].theta;
  g_arm_state.joints[1].dh_params.theta = g_arm_state.joints[1].theta;
  g_arm_state.joints[2].dh_params.theta = g_arm_state.joints[2].theta;
  g_arm_state.joints[3].dh_params.theta = g_arm_state.joints[3].theta;
  g_arm_state.joints[4].dh_params.theta = g_arm_state.joints[4].theta;
  g_arm_state.joints[5].dh_params.theta = g_arm_state.joints[5].theta;
}

void dh_update_arm_sim() {
  g_arm_state.sim_joints[0].dh_params.theta = g_arm_state.sim_joints[0].theta;
  g_arm_state.sim_joints[1].dh_params.theta = g_arm_state.sim_joints[1].theta;
  g_arm_state.sim_joints[2].dh_params.theta = g_arm_state.sim_joints[2].theta;
  g_arm_state.sim_joints[3].dh_params.theta = g_arm_state.sim_joints[3].theta;
  g_arm_state.sim_joints[4].dh_params.theta = g_arm_state.sim_joints[4].theta;
  g_arm_state.sim_joints[5].dh_params.theta = g_arm_state.sim_joints[5].theta;

}

void forward_kinematics(void) {
  dh_update_arm();

  for (int i = 0; i < 6; i++) {
    set_diag_const(g_arm_state.transforms[0], 1);
    assert(g_arm_state.transforms[i] != NULL);
  }

  for (int i = 0; i < 6; i++) {
    joint_transform_buffer(g_arm_state.joints[i], g_arm_state.buffer1_4x4);
    mat_mult_buffer(g_arm_state.transforms[i], g_arm_state.buffer1_4x4, g_arm_state.transforms[i+1]);
    g_arm_state.joints[i].x_pos = MAT_IDX(g_arm_state.transforms[i+1], 0, 3);
    g_arm_state.joints[i].y_pos = MAT_IDX(g_arm_state.transforms[i+1], 1, 3);
    g_arm_state.joints[i].z_pos = MAT_IDX(g_arm_state.transforms[i+1], 2, 3);
  }
}

void forward_kinematics_sim(float * theta_perturb) {
  dh_update_arm_sim();

  for (int i = 0; i < 6; i++) {
    set_diag_const(g_arm_state.sim_transforms[0], 1);

    assert(g_arm_state.sim_transforms[i] != NULL);
  }
  float old_theta[6];

  for (int i = 0; i < 6; i++) {
    old_theta[i] = g_arm_state.sim_joints[i].dh_params.theta;
    g_arm_state.sim_joints[i].dh_params.theta = theta_perturb[i];

    joint_transform_buffer(g_arm_state.sim_joints[i], g_arm_state.buffer1_4x4);
    mat_mult_buffer(g_arm_state.sim_transforms[i], g_arm_state.buffer1_4x4, g_arm_state.sim_transforms[i+1]);
    g_arm_state.sim_joints[i].x_pos = MAT_IDX(g_arm_state.sim_transforms[i+1], 0, 3);
    g_arm_state.sim_joints[i].y_pos = MAT_IDX(g_arm_state.sim_transforms[i+1], 1, 3);
    g_arm_state.sim_joints[i].z_pos = MAT_IDX(g_arm_state.sim_transforms[i+1], 2, 3);
  }

  for (int i = 0; i < 6; i++) {
    g_arm_state.sim_joints[i].dh_params.theta = old_theta[i];
  }
}

float clamp_val(float val, float min, float max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

void clamp_theta(float* theta) {
  for (int i = 0; i < 6; i++) {
    theta[i] = clamp_val(theta[i], MAT_IDX(g_arm_state.theta_lims, i, 0), MAT_IDX(g_arm_state.theta_lims, i, 1));
  }
}

void compute_orientation_error(Mat* R_current, Mat* R_target, Vec* rotvec_out) {
  mat_transpose_buffer(R_current, g_arm_state.buffer2_3x3);
  mat_mult_buffer(R_target, g_arm_state.buffer2_3x3, g_arm_state.buffer3_3x3);

  float trace = mat_trace(g_arm_state.buffer3_3x3);
  float theta = acos(clamp_val((trace - 1.0f) / 2.0f, -1.0f, 1.0f));
  float s = sin(theta);

  if (fabs(theta) < 1e-6) {
    VEC_IDX(rotvec_out, 0) = 0.0f;
    VEC_IDX(rotvec_out, 1) = 0.0f;
    VEC_IDX(rotvec_out, 2) = 0.0f;
    return;
  }

  if (fabs(s) < 1e-5) {
    s = 1e-5f;
  }

  VEC_IDX(rotvec_out, 0) = (MAT_IDX(g_arm_state.buffer3_3x3, 2, 1) - MAT_IDX(g_arm_state.buffer3_3x3, 1, 2)) / (2.0f * s);
  VEC_IDX(rotvec_out, 1) = (MAT_IDX(g_arm_state.buffer3_3x3, 0, 2) - MAT_IDX(g_arm_state.buffer3_3x3, 2, 0)) / (2.0f * s);
  VEC_IDX(rotvec_out, 2) = (MAT_IDX(g_arm_state.buffer3_3x3, 1, 0) - MAT_IDX(g_arm_state.buffer3_3x3, 0, 1)) / (2.0f * s);

  mat_scalar_mult_buffer(rotvec_out, theta, rotvec_out);
}

void extract_rot_matrix(Mat* T, Mat* R_out) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      MAT_IDX(R_out, i, j) = MAT_IDX(T, i, j);
    }
  }
}

void extract_pos_vector(Mat* T, Vec* pos_out) {
  VEC_IDX(pos_out, 0) = MAT_IDX(T, 0, 3);
  VEC_IDX(pos_out, 1) = MAT_IDX(T, 1, 3);
  VEC_IDX(pos_out, 2) = MAT_IDX(T, 2, 3);
}

float check_self_collision_dist(Mat **transforms) { 
  // transforms[5] and transforms[1] are already Mat* pointers!
  float dx = MAT_IDX(transforms[5], 0, 3) - MAT_IDX(transforms[1], 0, 3);
  float dy = MAT_IDX(transforms[5], 1, 3) - MAT_IDX(transforms[1], 1, 3);
  float dz = MAT_IDX(transforms[5], 2, 3) - MAT_IDX(transforms[1], 2, 3);

  return sqrtf(dx*dx + dy*dy + dz*dz);
}

void calculate_5d_jacobian_numerical_gradient(float damping, float * out_col_grad, float * out_current_col_dist) {
  Mat * J = g_arm_state.jacobian; // 5 x 6
  set_zero_mat(J);

  forward_kinematics();
  Mat* T_end = g_arm_state.transforms[6];

  extract_pos_vector(T_end, g_arm_state.wrist_pos);
  extract_rot_matrix(T_end, g_arm_state.current_orientation_mat);
  *out_current_col_dist = check_self_collision_dist(g_arm_state.transforms);

  float y_c[3] = {MAT_IDX(T_end, 0, 1), MAT_IDX(T_end, 1, 1), MAT_IDX(T_end, 2, 1)};
  float z_c[3] = {MAT_IDX(T_end, 0, 2), MAT_IDX(T_end, 1, 2), MAT_IDX(T_end, 2, 2)};

  float weight_pos = 1.0f;
  float weight_ori = 0.5f;

  float epsilon = 1e-4f;

  for (int i = 0; i < 6; i++) {
    float theta_perturb[6];
    for (int j = 0; j < 6; j++) {
      theta_perturb[j] = VEC_IDX(g_arm_state.theta, j);
    }    
    theta_perturb[i] += epsilon;

    forward_kinematics_sim(theta_perturb);
    Mat* T_perturb = g_arm_state.sim_transforms[6];

    // position gradient
    MAT_IDX(J, 0, i) = ((MAT_IDX(T_perturb, 0, 3) - VEC_IDX(g_arm_state.wrist_pos, 0)) / epsilon) * weight_pos;
    MAT_IDX(J, 1, i) = ((MAT_IDX(T_perturb, 1, 3) - VEC_IDX(g_arm_state.wrist_pos, 1)) / epsilon) * weight_pos;
    MAT_IDX(J, 2, i) = ((MAT_IDX(T_perturb, 2, 3) - VEC_IDX(g_arm_state.wrist_pos, 2)) / epsilon) * weight_pos;

    extract_rot_matrix(T_perturb, g_arm_state.buffer1_3x3);
    compute_orientation_error(g_arm_state.current_orientation_mat, g_arm_state.buffer1_3x3, g_arm_state.buffer1_3x1); // stores omega in buffer1_3x1
                                                                                                        // 

    float omega_x = VEC_IDX(g_arm_state.buffer1_3x1, 0) / epsilon;
    float omega_y = VEC_IDX(g_arm_state.buffer1_3x1, 1) / epsilon;
    float omega_z = VEC_IDX(g_arm_state.buffer1_3x1, 2) / epsilon;

    MAT_IDX(J, 3, i) = (y_c[0]*omega_x + y_c[1]*omega_y + y_c[2]*omega_z) * weight_ori;
    MAT_IDX(J, 4, i) = (z_c[0]*omega_x + z_c[1]*omega_y + z_c[2]*omega_z) * weight_ori;

    float perturbed_col_dist = check_self_collision_dist(g_arm_state.sim_transforms);
    out_col_grad[i] = (perturbed_col_dist - *out_current_col_dist) / epsilon;
  }

  // adaptive damped least squares

  Mat* J_T = mat_transpose_buffer(J, g_arm_state.buffer1_6x5);
  Mat* J_J_T = mat_mult_buffer(J, J_T, g_arm_state.buffer1_5x5);

  float w = sqrtf(fmaxf(0.0f, mat_determinant(J_J_T)));
  float w_threshold = 0.05f;
  float lambda_max = 0.1f;
  float lambda_val = 0.002f;

  if (w < w_threshold) {
    float ratio = w / w_threshold;
    lambda_val = lambda_max * (1.0f - (ratio * ratio));
  }

  Mat* I_5x5 = g_arm_state.buffer2_5x5;
  set_zero_mat(I_5x5);
  set_diag_const(I_5x5, lambda_val * lambda_val);

  mat_add_buffer(J_J_T, I_5x5, J_J_T);

  Mat* J_J_T_inv = mat_inverse_buffer(J_J_T, g_arm_state.buffer3_5x5);
  mat_mult_buffer(J_T, J_J_T_inv, g_arm_state.pseudo_inv_jacobian);

  Mat* J_pinv_J = mat_mult_buffer(g_arm_state.pseudo_inv_jacobian, J, g_arm_state.buffer1_6x6);
  Mat* I_6x6 = g_arm_state.buffer2_6x6;
  set_zero_mat(I_6x6);
  set_diag_const(I_6x6, 1.0f);
  mat_sub_buffer(I_6x6, J_pinv_J, g_arm_state.nullspace);
}

void inverse_kinematics(float* theta, Vec* target_pos, Mat* target_rot, int max_iters, float tol, float damping) {
  float col_grad[6];
  float current_col_dist;

  for (int iter = 0; iter < max_iters; iter++) {
    calculate_5d_jacobian_numerical_gradient(damping, col_grad, &current_col_dist);

    // pos erro
    float px_err = VEC_IDX(target_pos, 0) - VEC_IDX(g_arm_state.wrist_pos, 0);
    float py_err = VEC_IDX(target_pos, 1) - VEC_IDX(g_arm_state.wrist_pos, 1);
    float pz_err = VEC_IDX(target_pos, 2) - VEC_IDX(g_arm_state.wrist_pos, 2);

    float pos_norm = sqrtf(px_err * px_err + py_err * py_err + pz_err * pz_err);
    float max_pos_step = 0.03f;
    if (pos_norm > max_pos_step) {
      px_err *= (max_pos_step / pos_norm);
      py_err *= (max_pos_step / pos_norm);
      pz_err *= (max_pos_step / pos_norm);
    }

    // ori error
    compute_orientation_error(g_arm_state.current_orientation_mat, target_rot, g_arm_state.orientation_error);    
    float rx_err = VEC_IDX(g_arm_state.orientation_error, 0);
    float ry_err = VEC_IDX(g_arm_state.orientation_error, 1);
    float rz_err = VEC_IDX(g_arm_state.orientation_error, 2);

    float rot_norm = sqrtf(rx_err*rx_err + ry_err*ry_err + rz_err*rz_err);
    float max_rot_step = 0.1f;
    if (rot_norm > max_rot_step) {
      rx_err *= (max_rot_step / rot_norm);
      ry_err *= (max_rot_step / rot_norm);
      rz_err *= (max_rot_step / rot_norm);
    }

    // project rot error onto Y and Z axes
    float y_c[3] = {MAT_IDX(g_arm_state.current_orientation_mat, 0, 1), MAT_IDX(g_arm_state.current_orientation_mat, 1, 1), MAT_IDX(g_arm_state.current_orientation_mat, 2, 1)};
    float z_c[3] = {MAT_IDX(g_arm_state.current_orientation_mat, 0, 2), MAT_IDX(g_arm_state.current_orientation_mat, 1, 2), MAT_IDX(g_arm_state.current_orientation_mat, 2, 2)};

    float rot_y_proj = y_c[0] * rx_err + y_c[1] * ry_err + y_c[2] * rz_err;
    float rot_z_proj = z_c[0] * rx_err + z_c[1] * ry_err + z_c[2] * rz_err;

    Mat* error_5d = g_arm_state.buffer1_5x1;
    VEC_IDX(error_5d, 0) = px_err * 1.0f; // weight_pos
    VEC_IDX(error_5d, 1) = py_err * 1.0f;
    VEC_IDX(error_5d, 2) = pz_err * 1.0f;
    VEC_IDX(error_5d, 3) = rot_y_proj * 0.5f; // weight_ori
    VEC_IDX(error_5d, 4) = rot_z_proj * 0.5f;

    // joint limit avoidance nullspace objective

    Mat* q_null = g_arm_state.buffer1_6x1;
    set_zero_mat(q_null);

    float k_limit = 0.05f;
    float limit_margin = 0.2f;

    for (int i = 0; i < 6; i++) {
      float min_lim = MAT_IDX(g_arm_state.theta_lims, i, 0);
      float max_lim = MAT_IDX(g_arm_state.theta_lims, i, 1);

      if (theta[i] > max_lim - limit_margin) {
        float dist = (theta[i] - (max_lim - limit_margin)) / limit_margin;
        VEC_IDX(q_null, i) += -k_limit * (dist * dist);
      } else if (theta[i] < min_lim + limit_margin) {
        float dist = ((min_lim + limit_margin) - theta[i]) / limit_margin;
        VEC_IDX(q_null, i) += k_limit * (dist * dist);
      }
    }

    // collision avoidance nullspace objective

    float k_col = 1.0f;
    float safe_dist = 0.15f;
    if (current_col_dist < safe_dist) {
      float ratio = (safe_dist - current_col_dist) / safe_dist;
      float repulsion = k_col * (ratio * ratio);
      for (int i = 0; i < 6; i++){
        VEC_IDX(q_null, i) += repulsion * col_grad[i];
      }
    }

    // combind

    Mat* delta_primary = mat_mult_buffer(g_arm_state.pseudo_inv_jacobian, error_5d, g_arm_state.buffer2_6x1);
    Mat* delta_null = mat_mult_buffer(g_arm_state.nullspace, q_null, g_arm_state.buffer3_6x1);


    // apply and clip updates
    for (int i = 0; i < 6; i++) {
      float d_theta_prim = VEC_IDX(delta_primary, i);
      float d_theta_null = VEC_IDX(delta_null, i);
      
      d_theta_null = clamp_val(d_theta_null, -0.04f, 0.04f);

      float d_theta = d_theta_prim + d_theta_null;

      float min_lim = MAT_IDX(g_arm_state.theta_lims, i, 0);
      float max_lim = MAT_IDX(g_arm_state.theta_lims, i, 1);
      float margin = 0.15f;

      if (theta[i] > max_lim - margin && d_theta > 0) {
        d_theta *= clamp_val((max_lim - theta[i]) / margin, 0.0f, 1.0f);
      } else if (theta[i] < min_lim + margin && d_theta < 0) {
        d_theta *= clamp_val((theta[i] - min_lim) / margin, 0.0f, 1.0f);
      }

      d_theta = clamp_val(d_theta, -0.1f, 0.1f);
      theta[i] += d_theta;
    }
    /* float total_error = 0.0f;
    for (int i = 0; i < 5; i++) {
      total_error += fabsf(VEC_IDX(error_5d, i)); 
    } */

    clamp_theta(theta);

    for (int i = 0; i < 6; i++) {
      VEC_IDX(g_arm_state.theta, i) = theta[i];
      g_arm_state.joints[i].theta = theta[i];
    }

    forward_kinematics();

    px_err = VEC_IDX(target_pos, 0) - VEC_IDX(g_arm_state.wrist_pos, 0);
    py_err = VEC_IDX(target_pos, 1) - VEC_IDX(g_arm_state.wrist_pos, 1);
    pz_err = VEC_IDX(target_pos, 2) - VEC_IDX(g_arm_state.wrist_pos, 2);
    
    compute_orientation_error(g_arm_state.current_orientation_mat, target_rot, g_arm_state.orientation_error);    
    rx_err = VEC_IDX(g_arm_state.orientation_error, 0);
    ry_err = VEC_IDX(g_arm_state.orientation_error, 1);
    rz_err = VEC_IDX(g_arm_state.orientation_error, 2);

    float y_c_new[3] = {MAT_IDX(g_arm_state.current_orientation_mat, 0, 1), MAT_IDX(g_arm_state.current_orientation_mat, 1, 1), MAT_IDX(g_arm_state.current_orientation_mat, 2, 1)};
    float z_c_new[3] = {MAT_IDX(g_arm_state.current_orientation_mat, 0, 2), MAT_IDX(g_arm_state.current_orientation_mat, 1, 2), MAT_IDX(g_arm_state.current_orientation_mat, 2, 2)};

    rot_y_proj = y_c_new[0] * rx_err + y_c_new[1] * ry_err + y_c_new[2] * rz_err;
    rot_z_proj = z_c_new[0] * rx_err + z_c_new[1] * ry_err + z_c_new[2] * rz_err;


    
    float error_norm = sqrtf(px_err * px_err + py_err * py_err +
                             pz_err * pz_err + rot_y_proj * rot_y_proj +
                             rot_z_proj * rot_z_proj);
    if (error_norm < tol) {
      break;
    }
  }
  for (int i = 0; i < 6; i++) {
    g_arm_state.joints[i].theta = theta[i];
    g_arm_state.joints[i].x_pos = MAT_IDX(g_arm_state.transforms[i+1], 0, 3);
    g_arm_state.joints[i].y_pos = MAT_IDX(g_arm_state.transforms[i+1], 1, 3);
    g_arm_state.joints[i].z_pos = MAT_IDX(g_arm_state.transforms[i+1], 2, 3);
  }
}

/* 
int main() {
    // 1. Initialize the arm
    init_arm_state(&g_arm_state);
    
    // Set some initial starting angles (e.g., a neutral "home" pose)
    float current_theta[6] = {0.0f, PI/4, -PI/2, 0.0f, -PI/4, 0.0f};
    for (int i = 0; i < 6; i++) {
        g_arm_state.joints[i].theta = current_theta[i];
        VEC_IDX(g_arm_state.theta, i) = current_theta[i];
    }
    
    // Update initial kinematics
    forward_kinematics();
   
    float start_x = MAT_IDX(g_arm_state.transforms[6], 0, 3);
    float start_y = MAT_IDX(g_arm_state.transforms[6], 1, 3);
    float start_z = MAT_IDX(g_arm_state.transforms[6], 2, 3);

    // 2. Open a CSV file for logging
    FILE *log_file = fopen("trajectory.csv", "w");
    if (log_file == NULL) {
        printf("Error opening file!\n");
        return 1;
    }
    
    // Write CSV Header
    fprintf(log_file, "TargetX,TargetY,TargetZ,ActualX,ActualY,ActualZ,J1,J2,J3,J4,J5,J6\n");

    // 3. Create a target orientation (keep it constant for this test)
    Mat* target_rot = new_eye(3); // Assume we want the end-effector facing straight forward
    
    // 4. Simulate a teleoperation trajectory (moving smoothly along the X axis)
    Vec* target_pos = new_vec(3);
    
    int steps = 100;
    
    for (int step = 0; step < steps; step++) {
        // Move X forward by 1mm per step
        VEC_IDX(target_pos, 0) = start_x + (step * 0.01f); 
        VEC_IDX(target_pos, 1) = start_y + (step * 0.002f);
        VEC_IDX(target_pos, 2) = start_z;
       
        float actual_x = MAT_IDX(g_arm_state.transforms[6], 0, 3);
        float actual_y = MAT_IDX(g_arm_state.transforms[6], 1, 3);
        float actual_z = MAT_IDX(g_arm_state.transforms[6], 2, 3);
        
*/