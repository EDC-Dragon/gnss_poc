#include "gnss_math.h"

#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include "gnss_types.h"

double gaussian_noise(double sigma)
{
    double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);

    return sigma * sqrt(-2.0 * log(u1)) * cos(2.0 * acos(-1.0) * u2);
}

#define GNSS_PI 3.14159265358979323846

// Mélange déterministe de 64 bits (SplitMix64)
static uint64_t mix64(uint64_t x)
{
    x += UINT64_C(0x9E3779B97F4A7C15);
    x = (x ^ (x >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
    x = (x ^ (x >> 27)) * UINT64_C(0x94D049BB133111EB);
    return x ^ (x >> 31);
}

// Convertit 53 bits aléatoires en un double dans ]0, 1[
static double uniform01(uint64_t x)
{
    return ((double)(x >> 11) + 0.5) / 9007199254740992.0;
}

// Gaussian noise at a given time for a satellite (AI Generated)
double gaussian_noise_at_time(
    const uint64_t satellite_id,
    const double time_s,
    const double sigma_m)
{
    // Une valeur différente par milliseconde
    int64_t epoch_ms = (int64_t)llround(time_s * 1000.0);

    // Graine déterministe satellite + époque
    uint64_t seed = mix64(
        mix64(satellite_id) ^ mix64((uint64_t)epoch_ms) ^ UINT64_C(0x123456789ABCDEF0));

    // Deux uniformes indépendantes
    double u1 = uniform01(mix64(seed));
    double u2 = uniform01(mix64(seed + 1));

    // Transformation de Box-Muller
    double z = sqrt(-2.0 * log(u1)) * cos(2.0 * GNSS_PI * u2);

    return sigma_m * z;
}

GnssStatus compute_pseudorange(
    const Vec3 satellite_position,
    const GnssPositionClockState *receiver,
    double *pseudorange_m)
{
    if (!receiver || !pseudorange_m)
        return GNSS_INVALID_ARGUMENT;

    double distance = vec3_distance(&satellite_position, &receiver->position);

    double evaluated_pseudorange = distance + receiver->clock_bias_m;

    if (!isfinite(evaluated_pseudorange))
        return GNSS_NUMERICAL_ERROR;

    *pseudorange_m = evaluated_pseudorange;
    return GNSS_OK;
}

double compute_residue(
    double measurement,
    double estimated_pseudorange)
{
    return measurement - estimated_pseudorange;
}

void matrix_vector_multiply(
    size_t rows,
    size_t cols,
    double A[rows][cols],
    double x[cols],
    double y[rows])
{
    for (size_t i = 0; i < rows; i++)
    {
        double sum = 0.0;
        for (size_t j = 0; j < cols; j++)
            sum += A[i][j] * x[j];
        y[i] = sum;
    }
}

void vector_matrix_multiply(
    size_t rows,
    size_t cols,
    double x[rows],
    double A[rows][cols],
    double y[cols])
{
    for (size_t j = 0; j < cols; j++)
    {
        double sum = 0.0;
        for (size_t i = 0; i < rows; i++)
            sum += x[i] * A[i][j];
        y[j] = sum;
    }
}

void matrix_xmtx(
    size_t rows_x,
    size_t cols_x,
    double X[rows_x][cols_x],
    double M[cols_x][cols_x],
    double R[rows_x][rows_x])
{
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
}

GnssStatus matrix_add(
    size_t rows,
    size_t cols,
    double A[rows][cols],
    double B[rows][cols],
    double R[rows][cols])
{
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

GnssStatus matrix_solve_3x3(
    double A[3][3],
    double b[3],
    double x[3])
{
    double M[3][4];

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
            M[i][j] = A[i][j];

        M[i][3] = b[i];
    }

    // gaussian elimination
    for (size_t k = 0; k < 3; k++)
    {
        size_t pivot_row = k;
        double max_value = fabs(M[k][k]);

        for (size_t i = k + 1; i < 3; i++)
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
            for (size_t j = k; j < 4; j++)
            {
                double temp = M[k][j];
                M[k][j] = M[pivot_row][j];
                M[pivot_row][j] = temp;
            }
        }

        // eliminate below pivot
        for (size_t i = k + 1; i < 3; i++)
        {
            double factor = M[i][k] / M[k][k];

            for (size_t j = k; j < 4; j++)
                M[i][j] -= factor * M[k][j];
        }
    }

    // back substitution
    double solution[3];

    for (int i = 2; i >= 0; i--)
    {
        double sum = M[i][3];

        for (int j = i + 1; j < 3; j++)
            sum -= M[i][j] * solution[j];

        solution[i] = sum / M[i][i];

        if (!isfinite(solution[i]))
            return GNSS_NUMERICAL_ERROR;
    }

    for (size_t i = 0; i < 3; i++)
        x[i] = solution[i];

    return GNSS_OK;
}

GnssStatus vector_copy(
    size_t n,
    double x[n],
    double y[n])
{
    for (size_t i = 0; i < n; i++)
        y[i] = x[i];

    return GNSS_OK;
}