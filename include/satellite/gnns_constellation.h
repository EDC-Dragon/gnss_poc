#pragma once

#include "gnss_types.h"
#include "satellite/gnss_satellite.h"
#include "satellite/gnss_satellite_types.h"

#define MAX_SATELLITES_PER_CONSTELLATION 64

typedef struct
{
    Satellite satellites[MAX_SATELLITES_PER_CONSTELLATION];
} GnssConstellation;

GnssConstellation *gnss_constellation_create();
void gnss_constellation_free(GnssConstellation *constellation);

void gnss_constellation_notify_ephemeris(GnssConstellation *constellation, SatelliteId satellite_id, GnssGpsEphemeris ephemeris);

Satellite *gnss_constellation_get_satellite(GnssConstellation *constellation, SatelliteId satellite_id);