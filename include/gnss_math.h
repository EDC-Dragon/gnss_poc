#pragma once

#include "gnss_types.h"
#include "gnss_satellite.h"
#include "receiver/gnss_receiver.h"

double gaussian_noise(double sigma);

double gaussian_noise_at_time(const uint64_t satellite_id, const double time_s, const double sigma_m);

GnssStatus compute_pseudorange(
    const Vec3 satellite_position,
    const GnssPositionClockState *receiver,
    double *pseudorange_m);

double compute_residue(
    double measurement,
    double estimated_pseudorange);

void matrix_vector_multiply(
    size_t rows,
    size_t cols,
    double A[rows][cols],
    double x[cols],
    double y[rows]);

void vector_matrix_multiply(
    size_t rows,
    size_t cols,
    double x[rows],
    double A[rows][cols],
    double y[cols]);

void matrix_xmtx(
    size_t rows_x,
    size_t cols_x,
    double X[rows_x][cols_x],
    double M[cols_x][cols_x],
    double R[rows_x][rows_x]);

GnssStatus matrix_add(
    size_t rows,
    size_t cols,
    double A[rows][cols],
    double B[rows][cols],
    double R[rows][cols]);

GnssStatus matrix_copy(
    size_t rows,
    size_t cols,
    double A[rows][cols],
    double R[rows][cols]);

GnssStatus matrix_identity(
    size_t n,
    double R[n][n],
    double val);

GnssStatus matrix_solve_3x3(
    double A[3][3],
    double b[3],
    double x[3]);

GnssStatus vector_copy(
    size_t n,
    double x[n],
    double y[n]);