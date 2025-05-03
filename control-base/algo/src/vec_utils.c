#include "vec_utils.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Mat *new_mat(int rows, int cols)
{
    Mat *mat = (Mat *)malloc(sizeof(Mat));
    
    if (!mat)
    {
        perror("Failed to allocate memory for matrix");
        exit(EXIT_FAILURE);
    }

    mat->rows = rows;
    mat->cols = cols;
    mat->data = (float *)malloc(rows * cols * sizeof(float));

    if (!mat->data)
    {
        perror("Failed to allocate memory for matrix data");
        free(mat);
        exit(EXIT_FAILURE);
    }

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
    if (m)
    {
        free(m->data);
        free(m);
    }
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

Mat *create_temp_mat(Mat *m)
{
    return mat_copy(m);
}

/**
 * @brief DO NOT USE RN, IT IS BROKEN
 * 
 * Execute a 5rfunction that returns a Mat* and free the arguments
 *
 * @param func The function to execute
 * @param ... The arguments to pass to the function
 *
 * @attention The function must take Mat* arguments and NULL-terminate the list.
 * Also any input matrices will be freed after the function is executed. If you want
 * to keep the input matrices, pass a copy using mat_copy(). Alternatively, use the
 * MAT_OP_FREE_INPUT() macro to automatically copy the input.
 *
 * @return The result of the function
 *
 * WARNING: Be very careful with this function. It is very easy to introduce memory leaks.
 */
Mat *mat_execute_and_free(Mat *(*func)(void *, ...), ...) {
    va_list args;
    va_start(args, func);

    Mat* arg_array[5];
    int arg_count = 0;
    void* current_arg; 

     while ((current_arg = va_arg(args, void*)) != NULL) {
        if (arg_count >= 5) {
            fprintf(stderr, "Too many arguments to mat_execute_and_free\n");
            va_end(args);
            return NULL;
        }
        arg_array[arg_count++] = (Mat*)current_arg;
    }

    va_end(args);

    Mat* result = NULL;

    switch (arg_count) {
        case 1: result = func(arg_array[0]); break;
        case 2: result = func(arg_array[0], arg_array[1]); break;
        case 3: result = func(arg_array[0], arg_array[1], arg_array[2]); break;
        case 4: result = func(arg_array[0], arg_array[1], arg_array[2], arg_array[3]); break;
        case 5: result = func(arg_array[0], arg_array[1], arg_array[2], arg_array[3], arg_array[4]); break;
        default:
            fprintf(stderr, "Unsupported number of arguments in mat_execute_and_free\n");
            return NULL; 
    }

    for (int i = 0; i < arg_count; i++) {
        free_mat(arg_array[i]);
    }

    return result;
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

Mat* mat_mult_buffer(Mat *m1, Mat *m2, Mat *product)
{
    // assert(m1->cols == m2->rows &&
    //        product->rows == m1->rows &&
    //        product->cols == m2->cols);
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
    assert(product->rows == m->rows && product->cols == m->cols);
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
    assert(m1->rows == m2->rows && m1->cols == m2->cols && sum->rows == m1->rows && sum->cols == m1->cols);
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
    assert(m1->rows == m2->rows && m1->cols == m2->cols && diff->rows == m1->rows && diff->cols == m1->cols);
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
    assert(m->rows == buffer->cols && m->cols == buffer->rows);
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
 * NOTE: THIS FUNCTION ONLY WORKS FOR SQUARE MATRICES.
 *
 * @param m The matrix to transpose (will be overwritten).
 *
 * @return The transposed matrix.
 */
Mat *mat_transpose_overwrite(Mat *m)
{
    float temp;
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
 */
float mat_determinant(Mat *m)
{
    assert(m->rows == m->cols);
    float det = 0.0f;
    Mat *temp = NULL;
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
    default:
        // submatrix for cofactor
        temp = new_mat(m->rows - 1, m->cols - 1);
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
                    MAT_IDX(temp, subi, subj) = MAT_IDX(m, i, k);
                    subj++;
                }
                subi++;
            }
            det += sign * MAT_IDX(m, 0, j) * mat_determinant(temp);
        }
        free_mat(temp);
        return det;
    }
}

float mat_cofactor(Mat *m, int i, int j)
{
    assert(m->rows == m->cols);

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
    assert(m->rows == buffer->rows && m->cols == buffer->cols);
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

    assert(det != 0); // make sure matrix is invertible

    mat_transpose_overwrite(cofactor_mat);
    mat_scalar_mult_buffer(cofactor_mat, 1.0f / det, cofactor_mat);
    return cofactor_mat;
}

Mat *mat_inverse_buffer(Mat *m, Mat *buffer)
{
    assert(m->rows == m->cols);
    assert(buffer->rows == m->rows && buffer->cols == m->cols);

    mat_cofactor_matrix_buffer(m, buffer);

    float det = 0.0f;
    for (int j = 0; j < m->cols; j++)
    {
        det += MAT_IDX(m, 0, j) * MAT_IDX(buffer, 0, j);
    }

    assert(det != 0); // make sure matrix is invertible

    mat_transpose_overwrite(buffer);
    mat_scalar_mult_buffer(buffer, 1.0f / det, buffer);
    return buffer;
}

Mat *mat_pseudo_inverse(Mat *m)
{
    Mat *m_t = mat_transpose(m);
    Mat *m_tm, *m_tm_inv, *m_p;

    if (m->rows >= m->cols)
    {
        // tall matrix: (A^T A)^-1 A^T
        m_tm = mat_mult(m_t, m);
        m_tm_inv = mat_inverse(m_tm);
        m_p = mat_mult(m_tm_inv, m_t);
    }
    else
    {
        // wide matrix: A^T (A A^T)^-1
        m_tm = mat_mult(m, m_t);
        m_tm_inv = mat_inverse(m_tm);
        m_p = mat_mult(m_t, m_tm_inv);
    }

    free_mat(m_t);
    free_mat(m_tm);
    free_mat(m_tm_inv);

    return m_p;
}

Mat *mat_damped_pseudo_inverse(Mat *m, float rho)
{
    Mat *m_t = mat_transpose(m);
    Mat *m_tm, *m_tm_inv, *m_p, *m_eye;

    if (m->rows >= m->cols)
    {
        // tall matrix: (A^T A + ρ²I)^-1 A^T
        m_tm = mat_mult(m_t, m);
        m_eye = new_eye(m_tm->rows);
    }
    else
    {
        // wide matrix: A^T (A A^T + ρ²I)^-1
        m_tm = mat_mult(m, m_t);
        m_eye = new_eye(m_tm->rows);
    }

    mat_scalar_mult_buffer(m_eye, rho * rho, m_eye);
    mat_add_buffer(m_tm, m_eye, m_tm);
    m_tm_inv = mat_inverse(m_tm);

    if (m->rows >= m->cols)
    {
        m_p = mat_mult(m_tm_inv, m_t);
    }
    else
    {
        m_p = mat_mult(m_t, m_tm_inv);
    }

    free_mat(m_t);
    free_mat(m_tm);
    free_mat(m_tm_inv);
    free_mat(m_eye);

    return m_p;
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
    MAT_IDX(transform, 0, 1) = cosf(dh.theta);
    MAT_IDX(transform, 0, 2) = -1 * sinf(dh.theta) * cosf(dh.alpha);
    MAT_IDX(transform, 0, 3) = sinf(dh.theta) * sinf(dh.alpha);
    MAT_IDX(transform, 0, 4) = dh.a * cosf(dh.theta);
    // second row
    MAT_IDX(transform, 1, 1) = sinf(dh.theta);
    MAT_IDX(transform, 1, 2) = cosf(dh.theta) * cosf(dh.alpha);
    MAT_IDX(transform, 1, 3) = cosf(dh.theta) * sinf(dh.alpha);
    MAT_IDX(transform, 1, 4) = dh.a * sinf(dh.theta);
    // third row
    MAT_IDX(transform, 2, 1) = 0;
    MAT_IDX(transform, 2, 2) = sinf(dh.alpha);
    MAT_IDX(transform, 2, 3) = cosf(dh.alpha);
    MAT_IDX(transform, 2, 4) = dh.d;
    // fourth row
    MAT_IDX(transform, 3, 1) = 0;
    MAT_IDX(transform, 3, 2) = 0;
    MAT_IDX(transform, 3, 3) = 0;
    MAT_IDX(transform, 3, 4) = 1;

    return transform;
}

Mat *dh_transform_buffer(DH_Params dh, Mat *buffer)
{
    assert(buffer->rows == 4 && buffer->cols == 4);
    // first row
    MAT_IDX(buffer, 0, 1) = cosf(dh.theta);
    MAT_IDX(buffer, 0, 2) = -1 * sinf(dh.theta) * cosf(dh.alpha);
    MAT_IDX(buffer, 0, 3) = sinf(dh.theta) * sinf(dh.alpha);
    MAT_IDX(buffer, 0, 4) = dh.a * cosf(dh.theta);
    // second row
    MAT_IDX(buffer, 1, 1) = sinf(dh.theta);
    MAT_IDX(buffer, 1, 2) = cosf(dh.theta) * cosf(dh.alpha);
    MAT_IDX(buffer, 1, 3) = cosf(dh.theta) * sinf(dh.alpha);
    MAT_IDX(buffer, 1, 4) = dh.a * sinf(dh.theta);
    // third row
    MAT_IDX(buffer, 2, 1) = 0;
    MAT_IDX(buffer, 2, 2) = sinf(dh.alpha);
    MAT_IDX(buffer, 2, 3) = cosf(dh.alpha);
    MAT_IDX(buffer, 2, 4) = dh.d;
    // fourth row
    MAT_IDX(buffer, 3, 1) = 0;
    MAT_IDX(buffer, 3, 2) = 0;
    MAT_IDX(buffer, 3, 3) = 0;
    MAT_IDX(buffer, 3, 4) = 1;

    return buffer;
}