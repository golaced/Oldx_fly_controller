#include <stdio.h>
#include <assert.h>

#include "matrix.h"
#include "cholesky.h"
#include "matrix_unittests.h"

/**
* \brief Tests matrix inversion using Cholesky decomposition
*/
void test_matrix_inverse()
{
    int result;

    // data buffer for the original and decomposed matrix
    matrix_data_t d[3 * 3] = { 1, 0.5, 0,
        0.5, 1, 0,
        0, 0, 1 };

    // data buffer for the inverted matrix
    matrix_data_t di[3 * 3] = { 0, 0, 0,
        0, 0, 0,
        0, 0, 0 };

    // prepare matrix structures
    matrix_t m, mi;

    // initialize the matrices
    matrix_init(&m, 3, 3, d);
    matrix_init(&mi, 3, 3, di);

    // decompose matrix to lower triangular
    result = cholesky_decompose_lower(&m);

    // invert matrix using lower triangular
    matrix_invert_lower(&m, &mi);

    // test the result
    matrix_data_t test = matrix_get(&mi, 1, 1);
    assert(test >= 1.3);
}

/*!
* \brief Tests column and row fetching
*/
void test_matrix_copy_cols_and_rows()
{

    // data buffer for the original and decomposed matrix
    matrix_data_t d[3 * 4] = { 1, 2, 3, 10,
        4, 5, 6, 11,
        7, 8, 9, 12 };

    // data buffer for the copied row
    matrix_data_t v[4] = { 0, 0, 0 };
    matrix_data_t *vp;

    // prepare matrix structure
    matrix_t m;

    // initialize the matrix
    matrix_init(&m, 3, 4, d);

    // fetch the column
    matrix_get_column_copy(&m, 2, v);
    assert(v[0] == 3);
    assert(v[1] == 6);
    assert(v[2] == 9);

    // fetch the row
    matrix_get_row_copy(&m, 2, v);
    assert(v[0] == 7);
    assert(v[1] == 8);
    assert(v[2] == 9);
    assert(v[3] == 12);

    // fetch the row
    matrix_get_row_pointer(&m, 2, &vp);
    assert(vp[0] == 7);
    assert(vp[1] == 8);
    assert(vp[2] == 9);
    assert(vp[3] == 12);
}

/*!
*  \brief Tests matrix multiplication
*/
void test_matrix_multiply_aux()
{
    matrix_data_t ad[3 * 3] = { 1, 0.5, 0,
        0.5, 1, 0,
        0, 0, 1 };

    matrix_data_t bd[3 * 4] = { 1, 2, 3, 4,
        5, 6, 7, 8,
        9, 10, 11, 12 };

    matrix_data_t cd[3 * 4] = { 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0 };

    matrix_data_t aux[3] = { 0, 0, 0 };

    // prepare matrix structures
    matrix_t a, b, c;

    // initialize the matrices
    matrix_init(&a, 3, 3, ad);
    matrix_init(&b, 3, 4, bd);
    matrix_init(&c, 3, 4, cd);

    // multiply
    matrix_mult(&a, &b, &c, aux);
    assert(cd[0] == 3.5);
    assert(cd[1] == 5);
    assert(cd[4] == 5.5);
    assert(cd[11] == 12);
}

/*!
*  \brief Tests matrix multiplication with transposed B
*/
void test_matrix_multiply_transb()
{
    matrix_data_t ad[3 * 3] = { 1, 0.5, 0,
        0.5, 1, 0,
        0, 0, 1 };

    matrix_data_t bd[3 * 3] = { 1, 2, 3,
        5, 6, 7,
        9, 10, 11 };

    matrix_data_t cd[3 * 3] = { 0, 0, 0,
        0, 0, 0,
        0, 0, 0 };

    // prepare matrix structures
    matrix_t a, b, c;

    // initialize the matrices
    matrix_init(&a, 3, 3, ad);
    matrix_init(&b, 3, 3, bd);
    matrix_init(&c, 3, 3, cd);

    // multiply
    matrix_mult_transb(&a, &b, &c);
    assert(cd[0] == 2);
    assert(cd[1] == 8);
    assert(cd[4] == 8.5);
    assert(cd[8] == 11);
}

/*!
*  \brief Tests matrix multiplication with transposed B
*/
void test_matrix_multscale_transb()
{
    matrix_data_t ad[3 * 3] = { 1, 0.5, 0,
        0.5, 1, 0,
        0, 0, 1 };

    matrix_data_t bd[3 * 3] = { 1, 2, 3,
        5, 6, 7,
        9, 10, 11 };

    matrix_data_t cd[3 * 3] = { 0, 0, 0,
        0, 0, 0,
        0, 0, 0 };

    // prepare matrix structures
    matrix_t a, b, c;

    // initialize the matrices
    matrix_init(&a, 3, 3, ad);
    matrix_init(&b, 3, 3, bd);
    matrix_init(&c, 3, 3, cd);

    // multiply
    matrix_multscale_transb(&a, &b, 2, &c);
    assert(cd[0] == 2 * 2);
    assert(cd[1] == 8 * 2);
    assert(cd[4] == 8.5 * 2);
    assert(cd[8] == 11 * 2);
}

/*!
*  \brief Tests matrix multiplication with transposed B
*/
void test_matrix_multadd_transb()
{
    matrix_data_t ad[3 * 3] = { 1, 0.5, 0,
        0.5, 1, 0,
        0, 0, 1 };

    matrix_data_t bd[3 * 3] = { 1, 2, 3,
        5, 6, 7,
        9, 10, 11 };

    matrix_data_t cd[3 * 3] = { 10, 20, 30,
        40, 50, 60,
        70, 80, 90 };

    // prepare matrix structures
    matrix_t a, b, c;

    // initialize the matrices
    matrix_init(&a, 3, 3, ad);
    matrix_init(&b, 3, 3, bd);
    matrix_init(&c, 3, 3, cd);

    // multiply
    matrix_multadd_transb(&a, &b, &c);
    assert(cd[0] == 2 + 10);
    assert(cd[1] == 8 + 20);
    assert(cd[4] == 8.5 + 50);
    assert(cd[8] == 11 + 90);
}

/*!
*  \brief Tests matrix multiplication
*/
void test_matrix_multiply_vector()
{
    matrix_data_t ad[3 * 3] = { 1, 0.5, 0,
        0.5, 1, 0,
        0, 0, 1 };

    matrix_data_t bd[3 * 1] = { 1, 2, 3 };

    matrix_data_t cd[3 * 1] = { 1, 2, 3 };

    // prepare matrix structures
    matrix_t a, b, c;

    // initialize the matrices
    matrix_init(&a, 3, 3, ad);
    matrix_init(&b, 3, 1, bd);
    matrix_init(&c, 3, 1, cd);

    // multiply
    matrix_mult_rowvector(&a, &b, &c);
    assert(cd[0] == 2);
    assert(cd[1] == 2.5);
    assert(cd[2] == 3);
}

/*!
*  \brief Tests matrix multiplication
*/
void test_matrix_multiplyadd_vector()
{
    matrix_data_t ad[3 * 3] = { 1, 0.5, 0,
        0.5, 1, 0,
        0, 0, 1 };

    matrix_data_t bd[3 * 1] = { 1, 2, 3 };

    matrix_data_t cd[3 * 1] = { 10, 20, 30 };

    // prepare matrix structures
    matrix_t a, b, c;

    // initialize the matrices
    matrix_init(&a, 3, 3, ad);
    matrix_init(&b, 3, 1, bd);
    matrix_init(&c, 3, 1, cd);

    // multiply
    matrix_multadd_rowvector(&a, &b, &c);
    assert(cd[0] == 12);
    assert(cd[1] == 22.5);
    assert(cd[2] == 33);
}

/*!
*  \brief Tests matrix addition with B
*/
void test_matrix_add_inplace()
{
    matrix_data_t ad[3 * 3] = { 1, 0.5, 0,
        0.5, 1, 0,
        0, 0, 1 };

    matrix_data_t bd[3 * 3] = { 1, 2, 3,
        5, 6, 7,
        9, 10, 11 };

    // prepare matrix structures
    matrix_t a, b;

    // initialize the matrices
    matrix_init(&a, 3, 3, ad);
    matrix_init(&b, 3, 3, bd);

    // multiply
    matrix_add_inplace(&a, &b);
    assert(ad[0] == 2);
    assert(ad[1] == 2.5);
    assert(ad[4] == 7);
    assert(ad[8] == 12);
}

/*!
*  \brief Tests matrix subtraction with B
*/
void test_matrix_sub_inplace_b()
{
    matrix_data_t ad[3 * 3] = { 1, 0.5, 0,
        0.5, 1, 0,
        0, 0, 1 };

    matrix_data_t bd[3 * 3] = { 1, 2, 3,
        5, 6, 7,
        9, 10, 11 };

    // prepare matrix structures
    matrix_t a, b;

    // initialize the matrices
    matrix_init(&a, 3, 3, ad);
    matrix_init(&b, 3, 3, bd);

    // multiply
    matrix_sub_inplace_b(&a, &b);
    assert(bd[0] == 1 - 1);
    assert(bd[1] == 0.5 - 2);
    assert(bd[4] == 1 - 6);
    assert(bd[8] == 1 - 11);
}

/*!
*  \brief Tests matrix subtraction with B
*/
void test_matrix_sub()
{
    matrix_data_t ad[3 * 3] = { 1, 0.5, 0,
        0.5, 1, 0,
        0, 0, 1 };

    matrix_data_t bd[3 * 3] = { 1, 2, 3,
        5, 6, 7,
        9, 10, 11 };

    // prepare matrix structures
    matrix_t a, b;

    // initialize the matrices
    matrix_init(&a, 3, 3, ad);
    matrix_init(&b, 3, 3, bd);

    // multiply
    matrix_sub(&a, &b, &a);
    assert(ad[0] == 1 - 1);
    assert(ad[1] == 0.5 - 2);
    assert(ad[4] == 1 - 6);
    assert(ad[8] == 1 - 11);
}

/*!
*  \brief Tests matrix copying
*/
void test_matrix_copy()
{
    matrix_data_t ad[3 * 3] = { 1, 0.5, 0,
        0.5, 1, 0,
        0, 0, 1 };

    matrix_data_t bd[3 * 3] = { 1, 2, 3,
        5, 6, 7,
        9, 10, 11 };

    // prepare matrix structures
    matrix_t a, b;

    // initialize the matrices
    matrix_init(&a, 3, 3, ad);
    matrix_init(&b, 3, 3, bd);

    // multiply
    matrix_copy(&b, &a);
    assert(ad[0] == 1);
    assert(ad[1] == 2);
    assert(ad[4] == 6);
    assert(ad[8] == 11);
}

/*!
* \brief Unit tests for matrix operations
*/
void matrix_unittests()
{
    test_matrix_inverse();
    test_matrix_copy_cols_and_rows();
    test_matrix_multiply_aux();
    test_matrix_multiply_transb();
    test_matrix_multscale_transb();
    test_matrix_multadd_transb();
    test_matrix_multiply_vector();
    test_matrix_multiplyadd_vector();
    test_matrix_add_inplace();
    test_matrix_sub_inplace_b();
    test_matrix_sub();
    test_matrix_copy();
}