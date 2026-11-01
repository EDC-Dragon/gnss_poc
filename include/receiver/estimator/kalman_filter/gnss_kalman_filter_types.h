#pragma once

#include "gnss_types.h"

typedef struct
{
    double time_s;
    double state[6]; // x, y, z, vx, vy, vz
    double covariance[6][6];
} GnssKalmanState;