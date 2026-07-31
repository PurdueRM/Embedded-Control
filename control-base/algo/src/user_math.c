#include "user_math.h"
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <float.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

// #define assert(cond) if (!(cond)) { printf("Assertion failed: %s\n", #cond); exit(1); }

/*
-------------------------------------------------------------
SECTION:  Macros
-------------------------------------------------------------
*/

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

/*
-------------------------------------------------------------
SECTION:	Matrix Creation Functions
-------------------------------------------------------------
*/

Mat *new_mat(int rows, int cols)
{
    Mat *mat = (Mat *)malloc(sizeof(Mat));
    mat->rows = rows;
    mat->cols = cols;
    mat->op_code = OP_SUCCESS;
    mat->data = (float *)malloc(rows * cols * sizeof(float));
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            MAT_IDX(mat, i, j) = 0;
        }
    }
    return mat;
}

Mat *new_eye(int size)
{
    Mat *mat = new_mat(size, size);
    for (int i = 0; i < size; i++)
    {
        MAT_IDX(mat, i, i) = 1;
    }
    return mat;
}

Mat *new_mat_buffer(int rows, int cols, float *buffer)
{
    Mat *mat = (Mat *)malloc(sizeof(Mat));
    mat->rows = rows;
    mat->cols = cols;
    mat->data = (float *)malloc(rows * cols * sizeof(float));
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            MAT_IDX(mat, i, j) = buffer[i * cols + j];
        }
    }
    return mat;
}

void free_mat(Mat *m)
{
    free(m->data);
    free(m);
}

Mat *mat_copy(Mat *m)
{
    Mat *copy = new_mat(m->rows, m->cols);
    for (int i = 0; i < m->rows; i++)
    {
        for (int j = 0; j < m->cols; j++)
        {
            MAT_IDX(copy, i, j) = MAT_IDX(m, i, j);
        }
    }
    return copy;
}

Mat *mat_copy_buffer(Mat *m, Mat *buffer)
{
    if (m->rows != buffer->rows || m->cols != buffer->cols) {
        buffer->op_code = OP_INCORRECT_DIM;
        return buffer;
    }
    
    for (int i = 0; i < m->rows; i++) {
        for (int j = 0; j < m->cols; j++) {
            MAT_IDX(buffer, i, j) = MAT_IDX(m, i, j);
        }
    }
    return buffer;
}

Mat* mat_submatrix(Mat* m1, int num_rows, int num_cols, int start_row, int start_col) {
    assert(start_row >= 0 && start_col >= 0);
    assert(start_row + num_rows <= m1->rows && start_col + num_cols <= m1->cols);
    assert(num_rows > 0 && num_cols > 0);

    Mat* submat = new_mat(num_rows, num_cols);
    for (int i = 0; i < num_rows; i++) {
        for (int j = 0; j < num_cols; j++) {
            MAT_IDX(submat, i, j) = MAT_IDX(m1, start_row + i, start_col + j);
        }
    }

    return submat;
}

Mat* mat_submatrix_buffer(Mat* m1, int start_row, int start_col, Mat* buffer) {
    // assert(start_row >= 0 && start_col >= 0);
    // assert(start_row + buffer->rows <= m1->rows && start_col + buffer->cols <= m1->cols);

    if (!(start_row >= 0 && start_col >= 0)) {
        buffer->op_code = OP_INCORRECT_DIM;
        return buffer;
    } else if (!(start_row + buffer->rows <= m1->rows && start_col + buffer->cols <= m1->cols)) {
        buffer->op_code = OP_INCORRECT_DIM;
        return buffer;
    }

    for (int i = 0; i < buffer->rows; i++) {
        for (int j = 0; j < buffer->cols; j++) {
            MAT_IDX(buffer, i, j) = MAT_IDX(m1, start_row + i, start_col + j);
        }
    }

    return buffer;
}

Mat* mat_concatenate(Mat* m1, Mat* m2, int axis) {
    assert(axis == 0 || axis == 1);
    assert(axis == 0 ? m1->cols == m2->cols : m1->rows == m2->rows);

    Mat* concat = new_mat(m1->rows + (axis == 0 ? m2->rows : 0), m1->cols + (axis == 1 ? m2->cols : 0));
    for (int i = 0; i < m1->rows; i++) {
        for (int j = 0; j < m1->cols; j++) {
            MAT_IDX(concat, i, j) = MAT_IDX(m1, i, j);
        }
    }
    for (int i = 0; i < m2->rows; i++) {
        for (int j = 0; j < m2->cols; j++) {
            MAT_IDX(concat, i + (axis == 0 ? m1->rows : 0), j + (axis == 1 ? m1->cols : 0)) = MAT_IDX(m2, i, j);
        }
    }

    return concat;
}

Mat* mat_concatenate_buffer(Mat* m1, Mat* m2, int axis, Mat* buffer) {
    // assert(axis == 0 || axis == 1);
    // assert(axis == 0 ? m1->cols == m2->cols : m1->rows == m2->rows);
    // assert(buffer->rows == m1->rows + (axis == 0 ? m2->rows : 0) && buffer->cols == m1->cols + (axis == 1 ? m2->cols : 0));

    if (!(axis == 0 || axis == 1)) {
        buffer->op_code = OP_INVALID_INPUT;
        return buffer;
    } else if (!(axis == 0 ? m1->cols == m2->cols : m1->rows == m2->rows)) {
        buffer->op_code = OP_INCORRECT_DIM;
        return buffer;
    } else if (!(buffer->rows == m1->rows + (axis == 0 ? m2->rows : 0) && buffer->cols == m1->cols + (axis == 1 ? m2->cols : 0))) {
        buffer->op_code = OP_INCORRECT_DIM;
        return buffer;
    }

    for (int i = 0; i < m1->rows; i++) {
        for (int j = 0; j < m1->cols; j++) {
            MAT_IDX(buffer, i, j) = MAT_IDX(m1, i, j);
        }
    }
    for (int i = 0; i < m2->rows; i++) {
        for (int j = 0; j < m2->cols; j++) {
            MAT_IDX(buffer, i + (axis == 0 ? m1->rows : 0), j + (axis == 1 ? m1->cols : 0)) = MAT_IDX(m2, i, j);
        }
    }

    return buffer;
}

void set_diag(Mat *m, Vec *v) {
    if (v->rows < m->rows) {
        m->op_code = OP_INCORRECT_DIM;
        return;
    }

    set_zero_mat(m);

    for (int i = 0; i < MIN(m->cols, m->rows); i++) {
        MAT_IDX(m, i, i) = VEC_IDX(v, i);
    }
}

// WARNING: Assumes len(v) >= min(dim(m))
void set_diag_array(Mat *m, float *v) {
    set_zero_mat(m);

    for (int i = 0; i < MIN(m->cols, m->rows); i++) {
        MAT_IDX(m, i, i) = v[i];
    }
}

void set_diag_const(Mat *m, float value) {
    set_zero_mat(m);

    for (int i = 0; i < MIN(m->cols, m->rows); i++) {
        MAT_IDX(m, i, i) = value;
    }
}

void set_zero_mat(Mat *m) {
    for (int i = 0; i < m->rows; i++) {
        for (int j = 0; j < m->cols; j++) {
            MAT_IDX(m, i, j) = 0;
        }
    }
}

/*
-------------------------------------------------------------
SECTION:	Matrix Helpers
-------------------------------------------------------------
*/

char *mat_to_string(Mat *m)
{
    // esdtimate size
    int buffer_size = 1000;
    char *str = (char *)malloc(buffer_size * sizeof(char));
    if (!str)
        return NULL;

    char temp[50]; // Temporary buffer for formatting each number
    snprintf(str, buffer_size, "Matrix %dx%d\n", m->rows, m->cols);

    // check largest and smallest -> see if need sci notation decision
    float max_val = 0.0, min_val = FLT_MAX; // __FLT_MAX__;
    for (int i = 0; i < m->rows * m->cols; i++)
    {
        if (fabsf(m->data[i]) > max_val)
            max_val = fabsf(m->data[i]);
        if (fabsf(m->data[i]) < min_val && m->data[i] != 0)
            min_val = fabsf(m->data[i]);
    }

    int use_sci = (max_val > 1e4 || min_val < 1e-3); // if large diff, use sci notation

    for (int i = 0; i < m->rows; i++)
    {
        strcat(str, "|");
        for (int j = 0; j < m->cols; j++)
        {
            float val = MAT_IDX(m, i, j);
            if (use_sci)
                snprintf(temp, sizeof(temp), " % .2e", val); // scientific notation
            else
                snprintf(temp, sizeof(temp), " %6.2f", val); // default

            strcat(str, temp);
        }
        strcat(str, " |\n");
    }

    return str;
}

void print_mat(Mat *m)
{
    char *str = mat_to_string(m);
    printf("\n%s\n", str);
    free(str);
}

bool mat_equal(Mat *m1, Mat *m2, float tol)
{
    if (m1->rows != m2->rows || m1->cols != m2->cols)
    {
        return false;
    }
    for (int i = 0; i < m1->rows * m1->cols; i++)
    {
        if (fabsf(m1->data[i] - m2->data[i]) > tol)
        {
            return false;
        }
    }
    return true;
}

/*
-------------------------------------------------------------
SECTION:	General Matrix Operations
-------------------------------------------------------------
*/

Mat *mat_mult(Mat *m1, Mat *m2)
{
    assert(m1->cols == m2->rows);
    Mat *product = new_mat(m1->rows, m2->cols);
    product->rows = m1->rows;
    product->cols = m2->cols;
    for (int i = 0; i < m1->rows; i++)
    {
        for (int j = 0; j < m2->cols; j++)
        {
            MAT_IDX(product, i, j) = 0;
            for (int k = 0; k < m1->cols; k++)
            {
                MAT_IDX(product, i, j) += MAT_IDX(m1, i, k) * MAT_IDX(m2, k, j);
            }
        }
    }
    return product;
}

Mat *mat_mult_buffer(Mat *m1, Mat *m2, Mat *product)
{
    // assert(m1->cols == m2->rows &&
    //        product->rows == m1->rows &&
    //        product->cols == m2->cols);

    if (!(m1->cols == m2->rows &&
        product->rows == m1->rows &&
        product->cols == m2->cols)) {
            product->op_code = OP_INCORRECT_DIM;
            return product;
    }

    for (int i = 0; i < m1->rows; i++)
    {
        for (int j = 0; j < m2->cols; j++)
        {
            MAT_IDX(product, i, j) = 0;
            for (int k = 0; k < m1->cols; k++)
            {
                MAT_IDX(product, i, j) += MAT_IDX(m1, i, k) * MAT_IDX(m2, k, j);
            }
        }
    }

    return product;
}

Mat *mat_scalar_mult(Mat *m, float scalar)
{
    Mat *product = new_mat(m->rows, m->cols);
    product->rows = m->rows;
    product->cols = m->cols;
    for (int i = 0; i < m->rows; i++)
    {
        for (int j = 0; j < m->cols; j++)
        {
            MAT_IDX(product, i, j) = MAT_IDX(m, i, j) * scalar;
        }
    }
    return product;
}

Mat *mat_scalar_mult_buffer(Mat *m, float scalar, Mat *product)
{
    // assert(product->rows == m->rows && product->cols == m->cols);

    if (!(product->rows == m->rows && product->cols == m->cols)) {
        product->op_code = OP_INCORRECT_DIM;
        return product;
    }


    for (int i = 0; i < m->rows; i++)
    {
        for (int j = 0; j < m->cols; j++)
        {
            MAT_IDX(product, i, j) = MAT_IDX(m, i, j) * scalar;
        }
    }

    return product;
}

Mat *mat_add(Mat *m1, Mat *m2)
{
    assert(m1->rows == m2->rows && m1->cols == m2->cols);
    Mat *sum = new_mat(m1->rows, m1->cols);
    sum->rows = m1->rows;
    sum->cols = m1->cols;
    for (int i = 0; i < m1->rows; i++)
    {
        for (int j = 0; j < m1->cols; j++)
        {
            MAT_IDX(sum, i, j) = MAT_IDX(m1, i, j) + MAT_IDX(m2, i, j);
        }
    }
    return sum;
}

Mat *mat_add_buffer(Mat *m1, Mat *m2, Mat *sum)
{
    // assert(m1->rows == m2->rows && m1->cols == m2->cols && sum->rows == m1->rows && sum->cols == m1->cols);
    
    if (!(m1->rows == m2->rows && m1->cols == m2->cols && sum->rows == m1->rows && sum->cols == m1->cols)) {
        sum->op_code = OP_INCORRECT_DIM;
        return sum;
    }

    
    for (int i = 0; i < m1->rows; i++)
    {
        for (int j = 0; j < m1->cols; j++)
        {
            MAT_IDX(sum, i, j) = MAT_IDX(m1, i, j) + MAT_IDX(m2, i, j);
        }
    }
    return sum;
}

Mat *mat_sub(Mat *m1, Mat *m2)
{
    assert(m1->rows == m2->rows && m1->cols == m2->cols);

    Mat *diff = new_mat(m1->rows, m1->cols);
    diff->rows = m1->rows;
    diff->cols = m1->cols;
    for (int i = 0; i < m1->rows; i++)
    {
        for (int j = 0; j < m1->cols; j++)
        {
            MAT_IDX(diff, i, j) = MAT_IDX(m1, i, j) - MAT_IDX(m2, i, j);
        }
    }
    return diff;
}

Mat *mat_sub_buffer(Mat *m1, Mat *m2, Mat *diff)
{
    // assert(m1->rows == m2->rows && m1->cols == m2->cols && diff->rows == m1->rows && diff->cols == m1->cols);
    
    if (!(m1->rows == m2->rows && m1->cols == m2->cols && diff->rows == m1->rows && diff->cols == m1->cols)) {
        diff->op_code = OP_INCORRECT_DIM;
        return diff;
    }

    for (int i = 0; i < m1->rows; i++)
    {
        for (int j = 0; j < m1->cols; j++)
        {
            MAT_IDX(diff, i, j) = MAT_IDX(m1, i, j) - MAT_IDX(m2, i, j);
        }
    }
    return diff;
}

/*
-------------------------------------------------------------
SECTION:	Advanced Matrix Operations
-------------------------------------------------------------
*/

Mat *mat_transpose(Mat *m)
{
    Mat *transposed = new_mat(m->cols, m->rows);
    for (int i = 0; i < m->rows; i++)
    {
        for (int j = 0; j < m->cols; j++)
        {
            MAT_IDX(transposed, j, i) = MAT_IDX(m, i, j);
        }
    }
    return transposed;
}

Mat *mat_transpose_buffer(Mat *m, Mat *buffer)
{
    // assert(m->rows == buffer->cols && m->cols == buffer->rows);

    if (!(m->rows == buffer->cols && m->cols == buffer->rows)) {
        buffer->op_code = OP_INCORRECT_DIM;
        return buffer;
    }

    for (int i = 0; i < m->rows; i++)
    {
        for (int j = 0; j < m->cols; j++)
        {
            MAT_IDX(buffer, j, i) = MAT_IDX(m, i, j);
        }
    }
    return buffer;
}

/**
 * Transpose the matrix in place.
 * NOTE: THIS FUNCTION ONLY WORKS FOR SQUARE MATRICES. IT COULD WORK ON NON SQUARE BUT I 
 * HAVEN'T GOTTEN TO IT YET
 *
 * @param m The matrix to transpose (will be overwritten).
 *
 * @return The transposed matrix.
 */
Mat *mat_transpose_overwrite(Mat *m)
{
    if (!(m->rows == m->cols)) {
        m->op_code = OP_INCORRECT_DIM;
        return m;
    }
    
    float temp;
    // TODO: make work for non square by changing row and col and swapping as appropriate
    for (int i = 0; i < m->rows; i++)
    {
        for (int j = i + 1; j < m->cols; j++)
        {
            temp = MAT_IDX(m, i, j);
            MAT_IDX(m, i, j) = MAT_IDX(m, j, i);
            MAT_IDX(m, j, i) = temp;
        }
    }
    return m;
}

/**
 * Calculate the determinant of the matrix.
 *
 * @param m The matrix to calculate the determinant of.
 *
 * @return The determinant of the matrix.
 *
 * Example:
 *
 * if m = |1 2 3|
 *        |4 5 6|
 *        |7 8 9|
 *
 * You would call: mat_determinant(m)
 * 
 * For embedded contexts, don't want assert so returns opcode and FLOAT max value on error
 */
float mat_determinant(Mat *m)
{
    // assert(m->rows == m->cols);

    if (!(m->rows == m->cols)) {
        m->op_code = OP_INCORRECT_DIM;
        return FLT_MAX;
    }

    float det = 0.0f;
    // Mat *temp = NULL;
    switch (m->rows)
    {
    case 1:
        return m->data[0];
    case 2:
        return m->data[0] * m->data[3] - m->data[1] * m->data[2];
    case 3:
        for (int i = 0; i < 3; i++)
        {
            det += MAT_IDX(m, 0, i) * MAT_IDX(m, 1, (i + 1) % 3) * MAT_IDX(m, 2, (i + 2) % 3);
            det -= MAT_IDX(m, 0, i) * MAT_IDX(m, 1, (i + 2) % 3) * MAT_IDX(m, 2, (i + 1) % 3);
        }
        return det;
    default: {
        // TODO: FIND A WAY TO NOT USE MALLOC
        // submatrix for cofactor
        // temp = new_mat(m->rows - 1, m->cols - 1);
        
        float temp_data[16];
        Mat temp;
        temp.rows=m->rows-1;
        temp.cols=m->rows-1;
        temp.data = temp_data;

        for (int j = 0; j < m->cols; j++)
        {
          float sign = (j % 2 == 0) ? 1.0f : -1.0f;

          int subi = 0;
            for (int i = 1; i < 4; i++)
            {
              int subj = 0;
              for (int k = 0; k < 4; k++)
              {
                if (k == j)
                  continue;
                MAT_IDX(&temp, subi, subj) = MAT_IDX(m, i, k);
                subj++;
              }
              subi++;
            }
            det += sign * MAT_IDX(m, 0, j) * mat_determinant(&temp);
          }
        }
        // free_mat(temp);
        return det;
    }
}

float mat_cofactor(Mat *m, int i, int j)
{
    // assert(m->rows == m->cols);

    if (!(m->rows == m->cols)) {
        m->op_code = OP_INCORRECT_DIM;
        return FLT_MAX;
    }

    Mat *submat = new_mat(m->rows - 1, m->cols - 1);
    int sub_i = 0, sub_j = 0;

    for (int row = 0; row < m->rows; row++)
    {
        if (row == i)
            continue;
        sub_j = 0;
        for (int col = 0; col < m->cols; col++)
        {
            if (col == j)
                continue;
            MAT_IDX(submat, sub_i, sub_j) = MAT_IDX(m, row, col);
            sub_j++;
        }
        sub_i++;
    }

    float cofactor = (1 - 2 * ((i + j) % 2)) * mat_determinant(submat);
    free_mat(submat);
    return cofactor;
}

Mat *mat_cofactor_matrix(Mat *m)
{
    Mat *cofactor_mat = new_mat(m->rows, m->cols);
    for (int i = 0; i < m->rows; i++)
    {
        for (int j = 0; j < m->cols; j++)
        {
            MAT_IDX(cofactor_mat, i, j) = mat_cofactor(m, i, j);
        }
    }
    return cofactor_mat;
}

Mat *mat_cofactor_matrix_buffer(Mat *m, Mat *buffer)
{
    // assert(m->rows == buffer->rows && m->cols == buffer->cols);

    if(!(m->rows == buffer->rows && m->cols == buffer->cols)) {
        buffer->op_code = OP_INCORRECT_DIM;
        return buffer;
    }

    for (int i = 0; i < m->rows; i++)
    {
        for (int j = 0; j < m->cols; j++)
        {
            MAT_IDX(buffer, i, j) = mat_cofactor(m, i, j);
        }
    }
    return buffer;
}

Mat *mat_adjoint(Mat *m)
{
    Mat *cofactor_mat = mat_cofactor_matrix(m);
    Mat *adjoint_mat = mat_transpose_overwrite(cofactor_mat);
    return adjoint_mat;
}

Mat *mat_adjoint_buffer(Mat *m, Mat *buffer)
{
    mat_cofactor_matrix_buffer(m, buffer);
    return mat_transpose_overwrite(buffer);
}

Mat *mat_inverse(Mat *m)
{
    assert(m->rows == m->cols);

    Mat *cofactor_mat = new_mat(m->rows, m->cols);

    mat_cofactor_matrix_buffer(m, cofactor_mat);
    float det = 0.0f;
    for (int j = 0; j < m->cols; j++)
    {
        det += MAT_IDX(m, 0, j) * MAT_IDX(cofactor_mat, 0, j);
    }

    if (det == 0)
    {
        return NULL;
    }
    // assert(det != 0); // make sure matrix is invertible

    mat_transpose_overwrite(cofactor_mat);
    mat_scalar_mult_buffer(cofactor_mat, 1.0f / det, cofactor_mat);
    return cofactor_mat;
}

Mat *mat_inverse_buffer(Mat *m, Mat *buffer)
{
//     assert(m->rows == m->cols);
//     assert(buffer->rows == m->rows && buffer->cols == m->cols);


    if (!(m->rows == m->cols)) {
        buffer->op_code = OP_INCORRECT_DIM;
        return buffer;
    } else if (!(buffer->rows == m->rows && buffer->cols == m->cols)) {
        buffer->op_code = OP_INCORRECT_DIM;
        return buffer;
    }

    mat_cofactor_matrix_buffer(m, buffer);

    float det = 0.0f;
    for (int j = 0; j < m->cols; j++)
    {
        det += MAT_IDX(m, 0, j) * MAT_IDX(buffer, 0, j);
    }

    // assert(det != 0); // make sure matrix is invertible

    if (det == 0) {
        buffer->op_code = OP_NONIVERTIBLE;
        return buffer;
    } // make sure matrix is invertible

    mat_transpose_overwrite(buffer);
    mat_scalar_mult_buffer(buffer, 1.0f / det, buffer);
    return buffer;
}

//TODO: make buffered versions of these:
Mat *mat_pseudo_inverse(Mat *m)
{
    Mat *m_t = mat_transpose(m);
    Mat *m_tm = mat_mult(m_t, m);
    Mat *m_tm_inv = mat_inverse(m_tm);
    Mat *m_p = mat_mult(m_tm_inv, m_t);
    free_mat(m_t);
    free_mat(m_tm);
    free_mat(m_tm_inv);

    return m_p;
}

Mat *mat_damped_pseudo_inverse(Mat *m, float rho)
{
    Mat *m_t = mat_transpose(m);
    Mat *m_tm = mat_mult(m, m_t);
    Mat *m_eye = new_eye(m_tm->rows);
    mat_scalar_mult_buffer(m_eye, powf(rho, 2), m_eye);
    mat_add_buffer(m_tm, m_eye, m_tm);
    Mat *m_tm_inv = mat_inverse(m_tm);
    Mat *m_p = mat_mult(m_t, m_tm_inv);
    free_mat(m_t);
    free_mat(m_tm);
    free_mat(m_tm_inv);

    return m_p;
}

float mat_trace(Mat *m)
{
    assert(m->rows == m->cols);
    float trace = 0.0f;
    for (int i = 0; i < m->rows; i++)
    {
        trace += MAT_IDX(m, i, i);
    }
    return trace;
}

Mat* mat_clamp(Mat* val, float min, float max) {
    Mat* clamped = new_mat(val->rows, val->cols);
    for (int i = 0; i < val->rows * val->cols; i++) {
        clamped->data[i] = clamp(val->data[i], min, max);
    }
    return clamped;
}

Mat* mat_clamp_buffer(Mat* val, float min, float max, Mat* buffer) {
    assert(val->rows == buffer->rows && val->cols == buffer->cols);
    for (int i = 0; i < val->rows * val->cols; i++) {
        buffer->data[i] = clamp(val->data[i], min, max);
    }
    return buffer;
}

/*
-------------------------------------------------------------
SECTION:	VECTOR CREATION FUNCTIONS
-------------------------------------------------------------
*/

Vec *new_vec(int size)
{
    Vec *vec = new_mat(size, 1);
    return vec;
}

Vec *new_vec_buffer(int size, float *buffer)
{
    Vec *vec = new_mat_buffer(size, 1, buffer);
    return vec;
}

/*
-------------------------------------------------------------
SECTION:	VECTOR HELPERS
-------------------------------------------------------------
*/

bool assert_vec(Vec *v)
{
    bool cond = (v->cols == 1 && v->rows > 0);
    assert(cond);
    return cond;
}

/*
-------------------------------------------------------------
SECTION:	VECTOR OPERATIONS
-------------------------------------------------------------
*/

float vec_dot(Vec *v1, Vec *v2)
{
    assert(v1->rows == v2->rows);
    assert_vec(v1);
    assert_vec(v2);

    float dot = 0.0f;
    for (int i = 0; i < v1->rows; i++)
    {
        dot += v1->data[i] * v2->data[i];
    }
    return dot;
}

Vec *vec_cross(Vec *v1, Vec *v2)
{
    assert(v1->rows == 3 && v2->rows == 3);
    assert_vec(v1);
    assert_vec(v2);

    Vec *cross = new_vec(3);
    cross->data[0] = v1->data[1] * v2->data[2] - v1->data[2] * v2->data[1];
    cross->data[1] = v1->data[2] * v2->data[0] - v1->data[0] * v2->data[2];
    cross->data[2] = v1->data[0] * v2->data[1] - v1->data[1] * v2->data[0];
    return cross;
}

Vec *vec_cross_buffer(Vec *v1, Vec *v2, Vec *buffer)
{
    assert(v1->rows == 3 && v2->rows == 3 && buffer->rows == 3);
    assert_vec(v1);
    assert_vec(v2);
    assert_vec(buffer);

    buffer->data[0] = v1->data[1] * v2->data[2] - v1->data[2] * v2->data[1];
    buffer->data[1] = v1->data[2] * v2->data[0] - v1->data[0] * v2->data[2];
    buffer->data[2] = v1->data[0] * v2->data[1] - v1->data[1] * v2->data[0];
    return buffer;
}

float vec_magnitude(Vec *v)
{
    assert_vec(v);
    float mag = 0.0f;
    for (int i = 0; i < v->rows; i++)
    {
        mag += v->data[i] * v->data[i];
    }
    return sqrtf(mag);
}

Vec *vec_normalize(Vec *v)
{
    assert_vec(v);
    Vec *normalized = new_vec(v->rows);
    float mag = vec_magnitude(v);
    for (int i = 0; i < v->rows; i++)
    {
        normalized->data[i] = v->data[i] / mag;
    }
    return normalized;
}

Vec *vec_normalize_overwrite(Vec *v)
{
    assert_vec(v);
    float mag = vec_magnitude(v);
    for (int i = 0; i < v->rows; i++)
    {
        v->data[i] /= mag;
    }
    return v;
}

/*
-------------------------------------------------------------
SECTION:	DH TRANSFORMATIONS
-------------------------------------------------------------
*/

DH_Params *new_dh_params(float a, float alpha, float d, float theta)
{
    DH_Params *dh = (DH_Params *)malloc(sizeof(DH_Params));
    dh->a = a;
    dh->alpha = alpha;
    dh->d = d;
    dh->theta = theta;
    return dh;
}

void free_dh_params(DH_Params *dh)
{
    free(dh);
}

Mat *dh_transform(DH_Params dh)
{
    Mat *transform = new_mat(4, 4);
    // first row
    MAT_IDX(transform, 0, 0) = cosf(dh.theta);
    MAT_IDX(transform, 0, 1) = -1 * sinf(dh.theta) * cosf(dh.alpha);
    MAT_IDX(transform, 0, 2) = sinf(dh.theta) * sinf(dh.alpha);
    MAT_IDX(transform, 0, 3) = dh.a * cosf(dh.theta);
    // second row
    MAT_IDX(transform, 1, 0) = sinf(dh.theta);
    MAT_IDX(transform, 1, 1) = cosf(dh.theta) * cosf(dh.alpha);
    MAT_IDX(transform, 1, 2) = cosf(dh.theta) * sinf(dh.alpha);
    MAT_IDX(transform, 1, 3) = dh.a * sinf(dh.theta);
    // third row
    MAT_IDX(transform, 2, 0) = 0;
    MAT_IDX(transform, 2, 1) = sinf(dh.alpha);
    MAT_IDX(transform, 2, 2) = cosf(dh.alpha);
    MAT_IDX(transform, 2, 3) = dh.d;
    // fourth row
    MAT_IDX(transform, 3, 0) = 0;
    MAT_IDX(transform, 3, 1) = 0;
    MAT_IDX(transform, 3, 2) = 0;
    MAT_IDX(transform, 3, 3) = 1;

    return transform;
}

Mat *dh_transform_buffer(DH_Params dh, Mat *buffer)
{
    assert(buffer->rows == 4 && buffer->cols == 4);
    // first row
    MAT_IDX(buffer, 0, 0) = cosf(dh.theta);
    MAT_IDX(buffer, 0, 1) = -1 * sinf(dh.theta) * cosf(dh.alpha);
    MAT_IDX(buffer, 0, 2) = sinf(dh.theta) * sinf(dh.alpha);
    MAT_IDX(buffer, 0, 3) = dh.a * cosf(dh.theta);
    // second row
    MAT_IDX(buffer, 1, 0) = sinf(dh.theta);
    MAT_IDX(buffer, 1, 1) = cosf(dh.theta) * cosf(dh.alpha);
    MAT_IDX(buffer, 1, 2) = cosf(dh.theta) * sinf(dh.alpha);
    MAT_IDX(buffer, 1, 3) = dh.a * sinf(dh.theta);
    // third row
    MAT_IDX(buffer, 2, 0) = 0;
    MAT_IDX(buffer, 2, 1) = sinf(dh.alpha);
    MAT_IDX(buffer, 2, 2) = cosf(dh.alpha);
    MAT_IDX(buffer, 2, 3) = dh.d;
    // fourth row
    MAT_IDX(buffer, 3, 0) = 0;
    MAT_IDX(buffer, 3, 1) = 0;
    MAT_IDX(buffer, 3, 2) = 0;
    MAT_IDX(buffer, 3, 3) = 1;

    return buffer;
}

/*
-------------------------------------------------------------
SECTION:	GENERAL OPERATIONS
-------------------------------------------------------------
*/

float clamp(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}
