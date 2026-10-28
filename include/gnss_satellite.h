#pragma once

#include "gnss_types.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

typedef struct
{
    double bias_m;
    double noise_sigma_m;
} SatelliteErrorModel;

typedef struct
{
    SatelliteId id;
    Vec3 position;
    double time_s;
    SatelliteErrorModel error_model;
} Satellite;
