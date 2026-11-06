#pragma once

#include "gnss_core/gnss_types.h"
#include "gnss_core/receiver/estimator/kalman_filter/gnss_kalman_filter_types.h"

typedef struct GnssKalmanFilterInternal GnssKalmanFilterInternal;

GnssKalmanFilterInternal *gnss_kalman_filter_internal_create();
void gnss_kalman_filter_internal_free(GnssKalmanFilterInternal *kf);

GnssStatus gnss_kalman_filter_internal_update(GnssKalmanFilterInternal *kf, GnssKalmanState *previous_estimated_state, GnssPositionClockState *measurement);