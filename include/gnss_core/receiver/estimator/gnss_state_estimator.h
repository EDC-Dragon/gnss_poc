#pragma once

#include <stddef.h>

#include "gnss_core/gnss_types.h"
#include "gnss_state_estimator_types.h"

// GNSS state estimator, contains a Kalman filter and currently estimated state
typedef struct GnssStateEstimator GnssStateEstimator;

// Allocation
GnssStateEstimator *gnss_state_estimator_create();
// Deallocation
void gnss_state_estimator_free(GnssStateEstimator *estimator);

// Initialization
void gnss_state_estimator_initialize(GnssStateEstimator *estimator);

// Update the state estimator with a new measurement
// Returns:
//      GNSS_STATUS_OK if the update was successful
//      GNSS_STATUS_X otherwise
GnssStatus gnss_state_estimator_update(GnssStateEstimator *estimator, GnssPositionClockState *measurement);

// Get the currently estimated state
// Returns:
//      Pointer to the currently estimated state (never NULL)
GnssEstimatedState *gnss_state_estimator_get_estimated_state(GnssStateEstimator *estimator);
