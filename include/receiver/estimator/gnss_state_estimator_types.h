#pragma once

#include "gnss_types.h"

typedef struct
{
    Vec3 position;
    Vec3 velocity;
    double clock_bias_m;
    double time_s;
} GnssEstimatedState;

void gnss_estimated_state_print(const GnssEstimatedState *state);