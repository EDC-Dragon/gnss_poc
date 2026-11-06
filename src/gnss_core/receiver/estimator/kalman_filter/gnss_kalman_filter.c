#include "gnss_core/receiver/estimator/kalman_filter/gnss_kalman_filter.h"

#include <stdlib.h>
#include <stdio.h>

#include "gnss_kalman_filter_internal.h"
#include "tools/math/gnss_math_matrix.h"

struct GnssKalmanFilter
{
    bool state_initialized;
    GnssKalmanState estimated_state;

    GnssKalmanFilterInternal *internal;
};

GnssKalmanFilter *gnss_kalman_filter_create()
{
    GnssKalmanFilter *kf = malloc(sizeof(GnssKalmanFilter));
    if (!kf)
        return NULL;
    kf->internal = gnss_kalman_filter_internal_create();
    if (!kf->internal)
    {
        free(kf);
        return NULL;
    }
    return kf;
}

void gnss_kalman_filter_destroy(GnssKalmanFilter *kf)
{
    if (!kf)
        return;
    if (kf->internal)
        gnss_kalman_filter_internal_free(kf->internal);
    free(kf);
}

void gnss_kalman_filter_initialize(GnssKalmanFilter *kf)
{
    if (!kf)
        return;
    kf->state_initialized = false;
    kf->estimated_state.time_s = 0;
    for (int i = 0; i < 6; i++)
        kf->estimated_state.state[i] = 0;
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            kf->estimated_state.covariance[i][j] = 0;
}

void gnss_kalman_filter_print(GnssKalmanFilter *kf)
{
    if (!kf)
        return;
    printf("GnssKalmanFilter:\n");
    printf("  state_initialized: %s\n", kf->state_initialized ? "true" : "false");
    printf("  state: [");
    for (int i = 0; i < 6; i++)
        printf("%f ", kf->estimated_state.state[i]);
    printf("]\n");
    printf("  covariance:\n");
    for (int i = 0; i < 6; i++)
    {
        printf("    [");
        for (int j = 0; j < 6; j++)
            printf("%f ", kf->estimated_state.covariance[i][j]);
        printf("]\n");
    }
}

GnssStatus gnss_kalman_filter_initialize_state(GnssKalmanFilter *kalman_filter, const GnssPositionClockState *measurement)
{
    if (!kalman_filter || !measurement)
        return GNSS_INVALID_ARGUMENT;
    kalman_filter->estimated_state.state[0] = measurement->position.x;
    kalman_filter->estimated_state.state[1] = measurement->position.y;
    kalman_filter->estimated_state.state[2] = measurement->position.z;
    kalman_filter->estimated_state.state[3] = 0.0;
    kalman_filter->estimated_state.state[4] = 0.0;
    kalman_filter->estimated_state.state[5] = 0.0;
    kalman_filter->estimated_state.time_s = measurement->time_s;
    double initial_position_covariance = 100.0;
    double initial_velocity_covariance = 100.0;
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
        {
            if (i == j)
            {
                if (i < 3)
                    kalman_filter->estimated_state.covariance[i][j] = initial_position_covariance;
                else
                    kalman_filter->estimated_state.covariance[i][j] = initial_velocity_covariance;
            }
        }
    kalman_filter->state_initialized = true;
    return GNSS_OK;
}

GnssStatus gnss_kalman_filter_update(GnssKalmanFilter *kf, GnssPositionClockState *measurement)
{
    if (!kf || !measurement)
        return GNSS_INVALID_ARGUMENT;
    if (!kf->state_initialized)
        return gnss_kalman_filter_initialize_state(kf, measurement);
    else
    {
        return gnss_kalman_filter_internal_update(kf->internal, &kf->estimated_state, measurement);
    }
}

GnssKalmanState *gnss_kalman_filter_get_estimated_state(GnssKalmanFilter *kf)
{
    if (!kf)
        return NULL;
    return &kf->estimated_state;
}