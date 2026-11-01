#pragma once

#include "gnss_types.h"
#include "satellite/gnss_satellite.h"
#include "receiver/gnss_receiver.h"

GnssStatus matrix_vector_multiply(
    size_t rows,
    size_t cols,
    double A[rows][cols],
    double x[cols],
    double y[rows]);

GnssStatus vector_matrix_multiply(
    size_t rows,
    size_t cols,
    double x[rows],
    double A[rows][cols],
    double y[cols]);

GnssStatus matrix_xmtx(
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

GnssStatus matrix_solve(
    size_t n,
    double A[n][n],
    double b[n],
    double x[n]);

GnssStatus matrix_inverse(
    size_t n,
    double A[n][n],
    double inverse[n][n]);

GnssStatus matrix_square_submatrix(
    size_t n,
    double A[n][n],
    size_t sub_n,
    double sub[sub_n][sub_n]);

GnssStatus vector_copy(
    size_t n,
    double x[n],
    double y[n]);