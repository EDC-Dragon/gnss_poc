#include "receiver/estimator/kalman_filter/gnss_kalman_filter.h"

#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "gnss_math.h"

struct GnssKalmanFilter
{
    bool state_initialized;
    double state[6]; // x, y, z, vx, vy, vz
    double covariance[6][6];
    double time_s;

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

GnssKalmanFilter *gnss_kalman_filter_create()
{
    GnssKalmanFilter *kf = malloc(sizeof(GnssKalmanFilter));
    gnss_kalman_filter_initialize(kf);
    return kf;
}

void gnss_kalman_filter_destroy(GnssKalmanFilter *kf)
{
    free(kf);
}

void gnss_kalman_filter_initialize(GnssKalmanFilter *kf)
{
    kf->state_initialized = false;
    kf->time_s = 0;
    for (int i = 0; i < 6; i++)
        kf->state[i] = 0;
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            kf->covariance[i][j] = 0;
}

void gnss_kalman_filter_print(GnssKalmanFilter *kf)
{
    printf("GnssKalmanFilter:\n");
    printf("  state_initialized: %s\n", kf->state_initialized ? "true" : "false");
    printf("  state: [");
    for (int i = 0; i < 6; i++)
        printf("%f ", kf->state[i]);
    printf("]\n");
    printf("  covariance:\n");
    for (int i = 0; i < 6; i++)
    {
        printf("    [");
        for (int j = 0; j < 6; j++)
            printf("%f ", kf->covariance[i][j]);
        printf("]\n");
    }
}

void gnss_kalman_filter_initialize_state(GnssKalmanFilter *kalman_filter, const GnssPositionClockState *measurement)
{
    kalman_filter->state[0] = measurement->position.x;
    kalman_filter->state[1] = measurement->position.y;
    kalman_filter->state[2] = measurement->position.z;
    kalman_filter->state[3] = 0.0;
    kalman_filter->state[4] = 0.0;
    kalman_filter->state[5] = 0.0;
    kalman_filter->time_s = measurement->time_s;
    double initial_position_covariance = 100.0;
    double initial_velocity_covariance = 100.0;
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
        {
            if (i == j)
            {
                if (i < 3)
                    kalman_filter->covariance[i][j] = initial_position_covariance;
                else
                    kalman_filter->covariance[i][j] = initial_velocity_covariance;
            }
        }
    kalman_filter->state_initialized = true;
}

GnssStatus gnss_kalman_filter_evaluate_prediction(GnssKalmanFilter *kf, double dt)
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
        kf->state,
        kf->predicted_state);

    // P- = F * P * F^T

    matrix_xmtx(
        6, 6,
        F,
        kf->covariance,
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

GnssStatus gnss_kalman_filter_evaluate_innovation(GnssKalmanFilter *kf, GnssPositionClockState *measurement)
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

GnssStatus gnss_kalman_filter_evaluate_gain(GnssKalmanFilter *kf)
{
    // K = P- * H^T * S^-1
    for (int i = 0; i < 6; i++)
    {
        double b[3] = {
            kf->predicted_covariance[i][0],
            kf->predicted_covariance[i][1],
            kf->predicted_covariance[i][2]};

        double solution[3];

        GnssStatus status = matrix_solve_3x3(
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

GnssStatus gnss_kalman_filter_correction(GnssKalmanFilter *kf)
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

GnssStatus gnss_kalman_filter_update(GnssKalmanFilter *kf, GnssPositionClockState *measurement)
{
    if (!kf->state_initialized)
        gnss_kalman_filter_initialize_state(kf, measurement);
    else
    {
        double dt = measurement->time_s - kf->time_s;

        if (!isfinite(dt) || dt <= 0.0)
            return GNSS_INVALID_ARGUMENT;

        printf("dt %f\n", dt);

        // Prediction step
        GnssStatus status = gnss_kalman_filter_evaluate_prediction(kf, dt);
        if (status != GNSS_OK)
            return status;

        // Innovation evaluation
        status = gnss_kalman_filter_evaluate_innovation(kf, measurement);
        if (status != GNSS_OK)
            return status;

        // Gain evaluation
        status = gnss_kalman_filter_evaluate_gain(kf);
        if (status != GNSS_OK)
            return status;

        // Correction
        status = gnss_kalman_filter_correction(kf);
        if (status != GNSS_OK)
            return status;

        // Apply update
        vector_copy(6, kf->corrected_state, kf->state);
        matrix_copy(6, 6, kf->corrected_covariance, kf->covariance);
        kf->time_s = measurement->time_s;
    }

    return GNSS_OK;
}