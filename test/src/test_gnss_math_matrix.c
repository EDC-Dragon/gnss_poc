#include "test_gnss_math_matrix.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "math/gnss_math_matrix.h"

#define EPSILON 1e-9

static void assert_near(double actual, double expected)
{
    assert(isfinite(actual));
    assert(fabs(actual - expected) < EPSILON);
}

static void test_matrix_solve()
{
    printf("Testing matrix_solve\n");
    // 2x + y = 5
    // x + 3y = 7
    // Solution : x = 1.6, y = 1.8

    double A[2][2] = {
        {2.0, 1.0},
        {1.0, 3.0}};

    double b[2] = {5.0, 7.0};
    double x[2] = {0};

    assert(matrix_solve(2, A, b, x) == GNSS_OK);

    assert_near(x[0], 1.6);
    assert_near(x[1], 1.8);

    // La matrice d'entrée ne doit pas être modifiée.
    assert_near(A[0][0], 2.0);
    assert_near(A[0][1], 1.0);
    assert_near(A[1][0], 1.0);
    assert_near(A[1][1], 3.0);
}

static void test_matrix_solve_pivot()
{
    printf("Testing matrix_solve_pivot\n");
    // Nécessite une permutation des lignes.
    double A[2][2] = {
        {0.0, 1.0},
        {2.0, 3.0}};

    double b[2] = {2.0, 8.0};
    double x[2] = {0};

    assert(matrix_solve(2, A, b, x) == GNSS_OK);

    assert_near(x[0], 1.0);
    assert_near(x[1], 2.0);
}

static void test_matrix_solve_singular()
{
    printf("Testing matrix_solve_singular\n");
    // Lignes linéairement dépendantes.
    double A[2][2] = {
        {1.0, 2.0},
        {2.0, 4.0}};

    double b[2] = {3.0, 6.0};
    double x[2] = {0};

    assert(matrix_solve(2, A, b, x) == GNSS_NUMERICAL_ERROR);
}

static void test_matrix_xmtx()
{
    printf("Testing matrix_xmtx\n");
    // R = X * M * X^T

    double X[2][2] = {
        {1.0, 2.0},
        {3.0, 4.0}};

    double M[2][2] = {
        {2.0, 0.0},
        {0.0, 3.0}};

    double R[2][2] = {0};

    assert(matrix_xmtx(2, 2, X, M, R) == GNSS_OK);

    // Résultat attendu :
    // [14  30]
    // [30  66]

    assert_near(R[0][0], 14.0);
    assert_near(R[0][1], 30.0);
    assert_near(R[1][0], 30.0);
    assert_near(R[1][1], 66.0);
}

static void test_matrix_square_submatrix()
{
    printf("Testing matrix_square_submatrix\n");
    double A[4][4] = {
        {1, 2, 3, 4},
        {5, 6, 7, 8},
        {9, 10, 11, 12},
        {13, 14, 15, 16}};

    double sub[3][3] = {0};

    assert(matrix_square_submatrix(4, A, 3, sub) == GNSS_OK);

    double expected[3][3] = {
        {1, 2, 3},
        {5, 6, 7},
        {9, 10, 11}};

    for (size_t i = 0; i < 3; i++)
        for (size_t j = 0; j < 3; j++)
            assert_near(sub[i][j], expected[i][j]);
}

static void test_matrix_square_submatrix_invalid()
{
    printf("Testing matrix_square_submatrix_invalid\n");

    double A[2][2] = {
        {1, 2},
        {3, 4}};

    double sub[3][3] = {0};

    assert(
        matrix_square_submatrix(2, A, 3, sub) == GNSS_INVALID_ARGUMENT);
}

static void test_matrix_inverse()
{
    printf("Testing matrix_inverse\n");
    double A[2][2] = {
        {1, 2},
        {3, 4}};

    double inv[2][2] = {0};

    assert(matrix_inverse(2, A, inv) == GNSS_OK);
    assert_near(inv[0][0], -2.0);
    assert_near(inv[0][1], 1.0);
    assert_near(inv[1][0], 1.5);
    assert_near(inv[1][1], -0.5);

    double B[2][2] = {0};

    assert(matrix_inverse(2, inv, B) == GNSS_OK);

    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            assert_near(B[i][j], A[i][j]);
}

static void test_matrix_multiply()
{
    // CHeck that A * A' = I
    printf("Testing matrix_multiply\n");
    double A[2][2] = {
        {1, 2},
        {3, 4}};

    double inv[2][2] = {0};

    assert(matrix_inverse(2, A, inv) == GNSS_OK);

    double C[2][2] = {0};
    assert(matrix_multiply(2, 2, A, inv, C) == GNSS_OK);
    assert_near(C[0][0], 1.0);
    assert_near(C[1][1], 1.0);
    assert_near(C[0][1], 0.0);
    assert_near(C[1][0], 0.0);
}

void test_matrix_maths()
{
    test_matrix_solve();
    test_matrix_solve_pivot();
    test_matrix_solve_singular();
    test_matrix_xmtx();
    test_matrix_square_submatrix();
    test_matrix_square_submatrix_invalid();
    test_matrix_inverse();
    test_matrix_multiply();

    printf("All GNSS math tests passed.\n");
}
