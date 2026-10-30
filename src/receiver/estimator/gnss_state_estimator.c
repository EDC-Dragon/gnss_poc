#include "receiver/estimator/gnss_state_estimator.h"

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "gnss_math.h"
#include "receiver/estimator/kalman_filter/gnss_kalman_filter.h"

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

    gnss_kalman_filter_update(estimator->kalman_filter, measurement);

    gnss_state_estimator_print(estimator);
    return GNSS_OK;
}

GnssEstimatedState *gnss_state_estimator_get_estimated_state(GnssStateEstimator *estimator)
{
    return &estimator->estimated_state;
}

void gnss_state_estimator_free(GnssStateEstimator *estimator)
{
    free(estimator);
}