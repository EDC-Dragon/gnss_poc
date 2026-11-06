#pragma once

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "gnss_core/gnss_types.h"
#include "gnss_core/satellite/gnss_satellite_types.h"

typedef struct
{
    double bias_m;
    double noise_sigma_m;
} SatelliteErrorModel;

typedef struct
{
    SatelliteId id;

    bool has_ephemeris;
    GnssGpsEphemeris ephemeris;

    Vec3 position;
    double time_s;

    SatelliteErrorModel error_model;

} Satellite;
