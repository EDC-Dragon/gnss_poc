#pragma once

#include <stdbool.h>

#include "gnss_types.h"

typedef struct GnssKalmanFilter GnssKalmanFilter;

GnssKalmanFilter *gnss_kalman_filter_create();

void gnss_kalman_filter_destroy(GnssKalmanFilter *kf);

void gnss_kalman_filter_initialize(GnssKalmanFilter *kf);

void gnss_kalman_filter_print(GnssKalmanFilter *kf);

GnssStatus gnss_kalman_filter_update(GnssKalmanFilter *kf, GnssPositionClockState *measurement);
