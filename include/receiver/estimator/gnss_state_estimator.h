#pragma once

#include <stddef.h>

#include "gnss_types.h"
#include "gnss_state_estimator_types.h"

typedef struct GnssStateEstimator GnssStateEstimator;

GnssStateEstimator *gnss_state_estimator_create();

void gnss_state_estimator_initialize(GnssStateEstimator *estimator);

GnssStatus gnss_state_estimator_update(GnssStateEstimator *estimator, const GnssPositionClockState *measurement);

GnssEstimatedState *gnss_state_estimator_get_estimated_state(GnssStateEstimator *estimator);

void gnss_state_estimator_free(GnssStateEstimator *estimator);