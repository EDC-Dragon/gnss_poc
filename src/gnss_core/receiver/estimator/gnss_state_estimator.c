#include "gnss_core/receiver/estimator/gnss_state_estimator.h"

#include <stdlib.h>
#include <stdio.h>

#include "tools/math/gnss_math.h"
#include "gnss_core/receiver/estimator/kalman_filter/gnss_kalman_filter.h"
#include "gnss_core/receiver/estimator/kalman_filter/gnss_kalman_filter_types.h"

struct GnssStateEstimator
{
    GnssKalmanFilter *kalman_filter; // pointer, anticpate IMM?

    GnssEstimatedState estimated_state;
};

void gnss_state_estimator_print(GnssStateEstimator *estimator)
{
    printf("GnssStateEstimator:\n");
    gnss_kalman_filter_print(estimator->kalman_filter);
}

GnssStateEstimator *gnss_state_estimator_create()
{
    GnssStateEstimator *estimator = (GnssStateEstimator *)malloc(sizeof(GnssStateEstimator));
    if (estimator == NULL)
    {
        return NULL;
    }
    estimator->kalman_filter = gnss_kalman_filter_create();
    if (estimator->kalman_filter == NULL)
    {
        free(estimator);
        return NULL;
    }
    return estimator;
}

void gnss_state_estimator_initialize(GnssStateEstimator *estimator)
{
    gnss_kalman_filter_initialize(estimator->kalman_filter);

    estimator->estimated_state.position = (Vec3){0.0, 0.0, 0.0};
    estimator->estimated_state.velocity = (Vec3){0.0, 0.0, 0.0};
    estimator->estimated_state.clock_bias_m = 0.0;
    estimator->estimated_state.time_s = 0.0;
}

GnssStatus gnss_state_estimator_update(GnssStateEstimator *estimator, GnssPositionClockState *measurement)
{
    if (estimator == NULL || measurement == NULL)
    {
        return GNSS_INVALID_ARGUMENT;
    }
    printf("     ------------------------         \n");

    GnssStatus kalman_status = gnss_kalman_filter_update(estimator->kalman_filter, measurement);
    if (kalman_status != GNSS_OK)
        return kalman_status;

    GnssKalmanState *kf_state = gnss_kalman_filter_get_estimated_state(estimator->kalman_filter);
    estimator->estimated_state.position = (Vec3){kf_state->state[0], kf_state->state[1], kf_state->state[2]};
    estimator->estimated_state.velocity = (Vec3){kf_state->state[3], kf_state->state[4], kf_state->state[5]};
    estimator->estimated_state.clock_bias_m = measurement->clock_bias_m;
    estimator->estimated_state.time_s = measurement->time_s;

    gnss_state_estimator_print(estimator);
    return GNSS_OK;
}

GnssEstimatedState *gnss_state_estimator_get_estimated_state(GnssStateEstimator *estimator)
{
    return &estimator->estimated_state;
}

void gnss_state_estimator_free(GnssStateEstimator *estimator)
{
    if (estimator->kalman_filter)
        gnss_kalman_filter_destroy(estimator->kalman_filter);
    free(estimator);
}