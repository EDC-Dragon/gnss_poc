#include "tools/math/gnss_math_matrix.h"

#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include "gnss_core/gnss_types.h"

GnssStatus matrix_vector_multiply(
    size_t rows,
    size_t cols,
    double A[rows][cols],
    double x[cols],
    double y[rows])
{
    if (!A || !x || !y)
        return GNSS_INVALID_ARGUMENT;
    if (rows == 0 || cols == 0)
        return GNSS_INVALID_ARGUMENT;
    for (size_t i = 0; i < rows; i++)
    {
        double sum = 0.0;
        for (size_t j = 0; j < cols; j++)
            sum += A[i][j] * x[j];
        y[i] = sum;
    }
    return GNSS_OK;
}

GnssStatus vector_matrix_multiply(
    size_t rows,
    size_t cols,
    double x[rows],
    double A[rows][cols],
    double y[cols])
{
    if (!x || !A || !y)
        return GNSS_INVALID_ARGUMENT;
    if (rows == 0 || cols == 0)
        return GNSS_INVALID_ARGUMENT;
    for (size_t j = 0; j < cols; j++)
    {
        double sum = 0.0;
        for (size_t i = 0; i < rows; i++)
            sum += x[i] * A[i][j];
        y[j] = sum;
    }
    return GNSS_OK;
}

GnssStatus matrix_multiply(
    size_t rows,
    size_t cols,
    double A[rows][cols],
    double B[cols][cols],
    double R[rows][cols])
{
    if (!A || !B || !R)
        return GNSS_INVALID_ARGUMENT;
    if (rows == 0 || cols == 0)
        return GNSS_INVALID_ARGUMENT;
    for (size_t i = 0; i < rows; i++)
        for (size_t j = 0; j < cols; j++)
            R[i][j] = 0.0;
    for (size_t i = 0; i < rows; i++)
        for (size_t j = 0; j < cols; j++)
            for (size_t k = 0; k < cols; k++)
                R[i][j] += A[i][k] * B[k][j];

    return GNSS_OK;
}
GnssStatus matrix_xmtx(
    size_t rows_x,
    size_t cols_x,
    double X[rows_x][cols_x],
    double M[cols_x][cols_x],
    double R[rows_x][rows_x])
{
    if (!X || !M || !R)
        return GNSS_INVALID_ARGUMENT;
    if (rows_x == 0 || cols_x == 0)
        return GNSS_INVALID_ARGUMENT;

    double temp[rows_x][cols_x];

    // temp = X * M
    for (size_t i = 0; i < rows_x; i++)
    {
        for (size_t j = 0; j < cols_x; j++)
        {
            double sum = 0.0;

            for (size_t k = 0; k < cols_x; k++)
                sum += X[i][k] * M[k][j];

            temp[i][j] = sum;
        }
    }

    // R = temp * X^T
    for (size_t i = 0; i < rows_x; i++)
    {
        for (size_t j = 0; j < rows_x; j++)
        {
            double sum = 0.0;

            for (size_t k = 0; k < cols_x; k++)
                sum += temp[i][k] * X[j][k];

            R[i][j] = sum;
        }
    }
    return GNSS_OK;
}

GnssStatus matrix_add(
    size_t rows,
    size_t cols,
    double A[rows][cols],
    double B[rows][cols],
    double R[rows][cols])
{
    if (!A || !B || !R)
        return GNSS_INVALID_ARGUMENT;
    if (rows == 0 || cols == 0)
        return GNSS_INVALID_ARGUMENT;

    for (size_t i = 0; i < rows; i++)
        for (size_t j = 0; j < cols; j++)
            R[i][j] = A[i][j] + B[i][j];

    return GNSS_OK;
}

GnssStatus matrix_copy(
    size_t rows,
    size_t cols,
    double A[rows][cols],
    double R[rows][cols])
{
    if (!A || !R)
        return GNSS_INVALID_ARGUMENT;
    if (rows == 0 || cols == 0)
        return GNSS_INVALID_ARGUMENT;

    for (size_t i = 0; i < rows; i++)
        for (size_t j = 0; j < cols; j++)
            R[i][j] = A[i][j];

    return GNSS_OK;
}
GnssStatus matrix_identity(
    size_t n,
    double R[n][n],
    double val)
{
    if (!R)
        return GNSS_INVALID_ARGUMENT;
    if (n == 0)
        return GNSS_INVALID_ARGUMENT;

    for (size_t i = 0; i < n; i++)
        for (size_t j = 0; j < n; j++)
        {
            if (i == j)
                R[i][j] = val;
            else
                R[i][j] = 0.0;
        }

    return GNSS_OK;
}

GnssStatus matrix_solve(
    size_t n,
    double A[n][n],
    double b[n],
    double x[n])
{
    if (!A || !b || !x)
        return GNSS_INVALID_ARGUMENT;
    if (n == 0)
        return GNSS_INVALID_ARGUMENT;

    double M[n][n + 1];

    for (size_t i = 0; i < n; i++)
    {
        for (size_t j = 0; j < n; j++)
            M[i][j] = A[i][j];

        M[i][n] = b[i];
    }

    // gaussian elimination
    for (size_t k = 0; k < n; k++)
    {
        size_t pivot_row = k;
        double max_value = fabs(M[k][k]);

        for (size_t i = k + 1; i < n; i++)
        {
            double value = fabs(M[i][k]);

            if (value > max_value)
            {
                max_value = value;
                pivot_row = i;
            }
        }

        if (!isfinite(max_value) || max_value < 1e-12)
            return GNSS_NUMERICAL_ERROR;

        // swap rows if needed
        if (pivot_row != k)
        {
            for (size_t j = k; j < n + 1; j++)
            {
                double temp = M[k][j];
                M[k][j] = M[pivot_row][j];
                M[pivot_row][j] = temp;
            }
        }

        // eliminate below pivot
        for (size_t i = k + 1; i < n; i++)
        {
            double factor = M[i][k] / M[k][k];

            for (size_t j = k; j < n + 1; j++)
                M[i][j] -= factor * M[k][j];
        }
    }

    // back substitution
    double solution[n];

    for (int i = n - 1; i >= 0; i--)
    {
        double sum = M[i][n];

        for (size_t j = i + 1; j < n; j++)
            sum -= M[i][j] * solution[j];

        solution[i] = sum / M[i][i];

        if (!isfinite(solution[i]))
            return GNSS_NUMERICAL_ERROR;
    }

    for (size_t i = 0; i < n; i++)
        x[i] = solution[i];

    return GNSS_OK;
}

GnssStatus matrix_inverse(
    size_t n,
    double A[n][n],
    double inverse[n][n])
{
    if (!A || !inverse)
        return GNSS_INVALID_ARGUMENT;
    if (n == 0)
        return GNSS_INVALID_ARGUMENT;

    for (size_t col = 0; col < n; col++)
    {
        double b[n];
        double x[n];

        for (size_t i = 0; i < n; i++)
            b[i] = 0;

        b[col] = 1.0;

        GnssStatus status = matrix_solve(n, A, b, x);

        if (status != GNSS_OK)
            return status;

        for (size_t row = 0; row < n; row++)
            inverse[row][col] = x[row];
    }

    return GNSS_OK;
}

GnssStatus matrix_square_submatrix(
    size_t n,
    double A[n][n],
    size_t sub_n,
    double sub[sub_n][sub_n])
{
    if (!A || !sub)
        return GNSS_INVALID_ARGUMENT;
    if (sub_n == 0 || sub_n > n)
        return GNSS_INVALID_ARGUMENT;

    for (size_t i = 0; i < sub_n; i++)
    {
        for (size_t j = 0; j < sub_n; j++)
        {
            sub[i][j] = A[i][j];
        }
    }

    return GNSS_OK;
}

GnssStatus vector_copy(
    size_t n,
    double x[n],
    double y[n])
{
    if (!x || !y)
        return GNSS_INVALID_ARGUMENT;

    for (size_t i = 0; i < n; i++)
        y[i] = x[i];

    return GNSS_OK;
}