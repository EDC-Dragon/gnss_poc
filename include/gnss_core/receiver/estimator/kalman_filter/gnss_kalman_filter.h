#pragma once

#include "gnss_core/gnss_types.h"
#include "gnss_core/receiver/estimator/kalman_filter/gnss_kalman_filter_types.h"

typedef struct GnssKalmanFilter GnssKalmanFilter;

// Allocation
GnssKalmanFilter *gnss_kalman_filter_create();
// Deallocation
void gnss_kalman_filter_destroy(GnssKalmanFilter *kf);

// Initialization (sets up the Kalman filter for use, called after create)
void gnss_kalman_filter_initialize(GnssKalmanFilter *kf);

// Print the current state of the Kalman filter
void gnss_kalman_filter_print(GnssKalmanFilter *kf);

// Update the Kalman filter with a new measurement
// Returns:
//      GNSS_STATUS_OK if the update was successful
//      GNSS_STATUS_X otherwise
GnssStatus gnss_kalman_filter_update(GnssKalmanFilter *kf, GnssPositionClockState *measurement);

// Get the estimated state from the Kalman filter
GnssKalmanState *gnss_kalman_filter_get_estimated_state(GnssKalmanFilter *kf);