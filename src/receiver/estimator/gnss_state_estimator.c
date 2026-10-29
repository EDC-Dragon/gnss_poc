#include "receiver/estimator/gnss_state_estimator.h"

#include <stdlib.h>

GnssStateEstimator *gnss_state_estimator_create()
{
    GnssStateEstimator *estimator = (GnssStateEstimator *)malloc(sizeof(GnssStateEstimator));
    if (estimator == NULL)
    {
        return NULL;
    }
    return estimator;
}

void gnss_state_estimator_initialize(GnssStateEstimator *estimator)
{
    estimator->estimated_state.position = (Vec3){0.0, 0.0, 0.0};
    estimator->estimated_state.velocity = (Vec3){0.0, 0.0, 0.0};
    estimator->estimated_state.clock_bias_m = 0.0;
    estimator->estimated_state.time_s = 0.0;
}

GnssStatus gnss_state_estimator_update(GnssStateEstimator *estimator, const GnssPositionClockState *measurement)
{
    if (estimator == NULL || measurement == NULL)
    {
        return GNSS_INVALID_ARGUMENT;
    }
    // Implement the update logic here
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