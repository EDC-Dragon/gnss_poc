#pragma once

#include "gnss_types.h"

typedef struct
{
    double bias_m;
    double noise_sigma_m;
} SatelliteErrorModel;

typedef struct
{
    Vec3 position;
    double time_s;
    SatelliteErrorModel error_model;
} Satellite;
