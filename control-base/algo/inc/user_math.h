#ifndef USER_MATH_H
#define USER_MATH_H

#ifndef PI
#define PI (3.1415926f)
#endif
#define PI_OVER_2 (PI / 2.0f)

#define __MAX_LIMIT(val, min, max)     \
    do                                 \
    {                                  \
        val = (val > max ? max : val); \
        val = (val < min ? min : val); \
    } while (0);

#define DEG_TO_RAD 3.14159f / 180.0f

#define __MAP(x, in_min, in_max, out_min, out_max)                              \
    do                                                                          \
    {                                                                           \
        if (x > in_max)                                                         \
        {                                                                       \
            x -= in_max;                                                        \
        }                                                                       \
        else if (x < -in_max)                                                   \
        {                                                                       \
            x += in_max;                                                        \
        }                                                                       \
        x = ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min); \
    } while (0);

#define __MAP_ANGLE_TO_UNIT_CIRCLE(x) \
    do                                \
    {                                 \
        while (x >= PI)           \
        {                             \
            x -= 2 * PI;          \
        }                             \
        while (x < -PI)           \
        {                             \
            x += 2 * PI;          \
        }                             \
    } while (0);

#define BUFFER_SIZE (500)
#define __MOVING_AVERAGE(buffer, index, update_value, count, sum, average) \
    do                                                                     \
    {                                                                      \
        if (count < BUFFER_SIZE)                                           \
        {                                                                  \
            buffer[index] = update_value;                                  \
            sum += update_value;                                           \
            (count)++;                                                     \
        }                                                                  \
        else                                                               \
        {                                                                  \
            sum -= buffer[index];                                          \
            buffer[index] = update_value;                                  \
            sum += update_value;                                           \
        }                                                                  \
        index = (index + 1) % BUFFER_SIZE;                                 \
        average = sum / count;                                             \
    } while (0);

#define __IS_TOGGLED(input, prev_input) \
    (input == 1 && prev_input == 0)

// transition means go from a different state to a target state
#define __IS_TRANSITIONED(input, prev_input, transition_val) \
    ((input == transition_val) && (prev_input != transition_val))

#define __SLEW_RATE_LIMIT(curr, delta, ramp) \
    curr = (1 - (ramp)) * (curr) + (ramp) * (delta)

// First order filter
#define __FIRST_ORDER_FILTER(filtering_target, new_value, alpha) \
    do                                                               \
    {                                                                \
        filtering_target = (alpha) * (new_value) + (1 - (alpha)) * (filtering_target); \
    } while (0);
#define __ABS(x)         \
    (x < 0 ? -1 * x : x) \

// floor for floats
#define __FLOOR_F(x)                                                                      \
    (x < 0 && __ABS(x - ((int) x)) > 0.0001 ? (float)((int) x) - 1.0 : (float)((int) x))  \

#define __MOD_F(a, b)           \
    a - b * __FLOOR_F((a / b))  \

// #define bool int
// #define true 1
// #define false 0

// #define MAT_IDX(m, i, j) ((m)->data[(i) * (m)->cols + (j)])
// #define VEC_IDX(m, i) ((m)->data[(i)])

// // Error Handling Enum
// typedef enum Linalg_Op_Code_e {
//     OP_SUCCESS,
//     OP_FAILURE,
//     OP_NONIVERTIBLE,
//     OP_INCORRECT_DIM,
//     OP_INVALID_INPUT
// } Linalg_Op_Code_e;


// /*
// -------------------------------------------------------------
// SECTION:	MATRICES
// -------------------------------------------------------------
// */

// // NxN Matrix struct
// typedef struct Mat {
//     int rows;
//     int cols;
//     float* data;
//     Linalg_Op_Code_e op_code;
// } Mat;

// // matrix creation functions
// Mat* new_mat(int rows, int cols);
// Mat* new_eye(int size);
// Mat* new_mat_buffer(int rows, int cols, float* buffer);
// void free_mat(Mat* m);
// Mat* mat_copy(Mat* m);
// Mat* create_temp_mat(Mat* m);
// Mat* mat_execute_and_free(Mat* (*func)(void *, ...), ...);
// Mat *mat_copy_buffer(Mat *m, Mat *buffer);
// Mat* mat_submatrix(Mat* m1, int num_rows, int num_cols, int start_row, int start_col);
// Mat* mat_submatrix_buffer(Mat* m1, int start_row, int start_col, Mat* buffer);
// Mat* mat_concatenate(Mat* m1, Mat* m2, int axis);
// Mat* mat_concatenate_buffer(Mat* m1, Mat* m2, int axis, Mat* buffer);
// void set_diag(Mat *m, Mat *v); // note Mat v is really a vector, I just don't want to move the macro
// void set_diag_array(Mat *m, float *v);
// void set_diag_const(Mat *m, float value);
// void set_zero_mat(Mat *m);

// // matrix helpers
// char* mat_to_string(Mat* m);
// void print_mat(Mat* m);
// bool mat_equal(Mat* m1, Mat* m2, float tol);

// // general matrix operations
// Mat* mat_mult(Mat* m1, Mat* m2);
// Mat* mat_mult_buffer(Mat* m1, Mat* m2, Mat* product);
// Mat* mat_scalar_mult(Mat* m, float scalar);
// Mat* mat_scalar_mult_buffer(Mat* m, float scalar, Mat* product);
// Mat* mat_add(Mat* m1, Mat* m2);
// Mat* mat_add_buffer(Mat* m1, Mat* m2, Mat* sum);
// Mat* mat_sub(Mat* m1, Mat* m2);
// Mat* mat_sub_buffer(Mat* m1, Mat* m2, Mat* diff);

// // advanced matrix operations
// float mat_determinant(Mat* m);
// Mat* mat_adjoint(Mat* m);
// Mat* mat_adjoint_buffer(Mat* m, Mat* buffer);
// float mat_cofactor(Mat *m, int i, int j);
// Mat* mat_cofactor_matrix(Mat* m);
// Mat* mat_cofactor_matrix_buffer(Mat* m, Mat* buffer);
// Mat* mat_inverse(Mat* m);
// Mat* mat_inverse_buffer(Mat* m, Mat* buffer);
// Mat* mat_transpose(Mat *m);
// Mat* mat_transpose_buffer(Mat *m, Mat* buffer);
// Mat* mat_transpose_overwrite(Mat *m);
// Mat* mat_pseudo_inverse(Mat *m);
// Mat* mat_damped_pseudo_inverse(Mat* m, float rho);
// float mat_trace(Mat *m);
// Mat* mat_clamp(Mat* val, float min, float max);
// Mat* mat_clamp_buffer(Mat* val, float min, float max, Mat* buffer);

// // TODO: ADD PSEUDO INVERSE

// // TODO: If I feel like it: LU DECOMPOSITION, QR DECOMPOSITION, SVD DECOMPOSITION, EIGEN DECOMPOSITION, SOLVE LINEAR SYSTEM, SOLVE EIGENVALUE PROBLEM, SOLVE EIGENVECTOR PROBLEM, SOLVE SVD PROBLEM

// /*
// -------------------------------------------------------------
// SECTION:	VECTORS
// -------------------------------------------------------------
// */

// // To maintain compatibility we use 1D matrix for vectors. You can simply use the matrix functions for vectors.
// #define Vec Mat

// // vector creation functions
// Vec* new_vec(int size);
// Vec* new_vec_buffer(int size, float* buffer);

// // vector helpers
// bool assert_vec(Vec* v);

// // general vector operations
// float vec_dot(Vec* v1, Vec* v2);
// Vec* vec_cross(Vec* v1, Vec* v2);
// Vec* vec_cross_buffer(Vec* v1, Vec* v2, Vec* buffer);
// float vec_magnitude(Vec* v);
// Vec* vec_normalize(Vec* v);
// Vec* vec_normalize_overwrite(Vec* v);


// /*
// -------------------------------------------------------------
// SECTION:	DH TRANSFORMATIONS
// -------------------------------------------------------------
// */

// typedef struct DH_Params {
//     float a;
//     float alpha;
//     float d;
//     float theta;
// } DH_Params;

// // Denavit-Hartenberg matrix calculations
// DH_Params* new_dh_params(float a, float alpha, float d, float theta);
// void free_dh_params(DH_Params* dh);
// Mat* dh_transform(DH_Params dh);
// Mat* dh_transform_buffer(DH_Params dh, Mat* buffer);


// /*
// -------------------------------------------------------------
// SECTION:	GENERAL OPERATIONS
// -------------------------------------------------------------
// */

// float clamp(float val, float min, float max);

#endif