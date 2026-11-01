#include "gnss_kalman_filter_internal.h"

#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "gnss_math.h"

struct GnssKalmanFilterInternal
{
    // Prediction
    double predicted_state[6];
    double predicted_covariance[6][6];

    // Measurement noise
    double R[3][3];

    // Innovation
    double innovation[3];
    double covariance_innovation[3][3];

    // Gain
    double gain[6][3];

    // Corrected state
    double corrected_state[6];
    double corrected_covariance[6][6];
};

GnssKalmanFilterInternal *gnss_kalman_filter_internal_create()
{
    GnssKalmanFilterInternal *kf = malloc(sizeof(GnssKalmanFilterInternal));
    return kf;
}

void gnss_kalman_filter_internal_free(GnssKalmanFilterInternal *kf)
{
    free(kf);
}

GnssStatus gnss_kalman_filter_internal_evaluate_prediction(GnssKalmanFilterInternal *kf, GnssKalmanState *previous_estimated_state, double dt)
{
    // State transition matrix F
    double F[6][6] = {0};

    for (int i = 0; i < 6; i++)
        F[i][i] = 1.0;

    F[0][3] = dt;
    F[1][4] = dt;
    F[2][5] = dt;

    // x- = F * x

    matrix_vector_multiply(
        6, 6,
        F,
        previous_estimated_state->state,
        kf->predicted_state);

    // P- = F * P * F^T

    matrix_xmtx(
        6, 6,
        F,
        previous_estimated_state->covariance,
        kf->predicted_covariance);

    double sigma_a = 2.0; // m/s²
    double q = sigma_a * sigma_a;

    double dt2 = dt * dt;
    double dt3 = dt2 * dt;
    double dt4 = dt3 * dt;

    for (int i = 0; i < 3; i++)
    {
        kf->predicted_covariance[i][i] += q * dt4 / 4.0;

        kf->predicted_covariance[i][i + 3] += q * dt3 / 2.0;
        kf->predicted_covariance[i + 3][i] += q * dt3 / 2.0;

        kf->predicted_covariance[i + 3][i + 3] += q * dt2;
    }

    return GNSS_OK;
}

GnssStatus gnss_kalman_filter_internal_evaluate_innovation(GnssKalmanFilterInternal *kf, GnssPositionClockState *measurement)
{
    matrix_copy(3, 3, measurement->covariance, kf->R);

    // y = z - H * x-
    kf->innovation[0] = measurement->position.x - kf->predicted_state[0];
    kf->innovation[1] = measurement->position.y - kf->predicted_state[1];
    kf->innovation[2] = measurement->position.z - kf->predicted_state[2];

    // S = H * P- * H^T + R

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            kf->covariance_innovation[i][j] =
                kf->predicted_covariance[i][j] + kf->R[i][j];
        }
    }

    return GNSS_OK;
}

GnssStatus gnss_kalman_filter_internal_evaluate_gain(GnssKalmanFilterInternal *kf)
{
    // K = P- * H^T * S^-1
    for (int i = 0; i < 6; i++)
    {
        double b[3] = {
            kf->predicted_covariance[i][0],
            kf->predicted_covariance[i][1],
            kf->predicted_covariance[i][2]};

        double solution[3];

        GnssStatus status = matrix_solve(
            3,
            kf->covariance_innovation,
            b,
            solution);

        if (status != GNSS_OK)
            return status;

        for (int j = 0; j < 3; j++)
            kf->gain[i][j] = solution[j];
    }
    return GNSS_OK;
}

GnssStatus gnss_kalman_filter_internal_correction(GnssKalmanFilterInternal *kf)
{
    // x+ = X- + K * innovation

    for (int i = 0; i < 6; i++)
    {
        kf->corrected_state[i] = kf->predicted_state[i];

        for (int j = 0; j < 3; j++)
        {
            kf->corrected_state[i] += kf->gain[i][j] * kf->innovation[j];
        }
    }

    // P+ = (I-KH) * P- * (I-KH)^T + K * R * K^T
    double IKH[6][6] = {0};

    for (int i = 0; i < 6; i++)
        IKH[i][i] = 1.0;

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            IKH[i][j] -= kf->gain[i][j];
        }
    }

    // P1 = (I-KH) * P- * (I-KH)^T
    double P1[6][6];

    matrix_xmtx(
        6, 6,
        IKH,
        kf->predicted_covariance,
        P1);

    // P2 = K * R * K^T
    double P2[6][6];

    matrix_xmtx(
        6, 3,
        kf->gain,
        kf->R,
        P2);

    matrix_add(6, 6, P1, P2, kf->corrected_covariance);

    return GNSS_OK;
}

GnssStatus gnss_kalman_filter_internal_update(GnssKalmanFilterInternal *kf, GnssKalmanState *previous_estimated_state, GnssPositionClockState *measurement)
{
    double dt = measurement->time_s - previous_estimated_state->time_s;

    if (!isfinite(dt) || dt <= 0.0)
        return GNSS_INVALID_ARGUMENT;

    // Prediction step
    GnssStatus status = gnss_kalman_filter_internal_evaluate_prediction(kf, previous_estimated_state, dt);
    if (status != GNSS_OK)
        return status;

    // Innovation evaluation
    status = gnss_kalman_filter_internal_evaluate_innovation(kf, measurement);
    if (status != GNSS_OK)
        return status;

    // Gain evaluation
    status = gnss_kalman_filter_internal_evaluate_gain(kf);
    if (status != GNSS_OK)
        return status;

    // Correction
    status = gnss_kalman_filter_internal_correction(kf);
    if (status != GNSS_OK)
        return status;

    // Apply update
    vector_copy(6, kf->corrected_state, previous_estimated_state->state);
    matrix_copy(6, 6, kf->corrected_covariance, previous_estimated_state->covariance);
    previous_estimated_state->time_s = measurement->time_s;

    return GNSS_OK;
}