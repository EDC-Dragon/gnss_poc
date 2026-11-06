#pragma once

#include "gnss_core/gnss_types.h"
#include "gnss_core/satellite/gnss_satellite.h"
#include "gnss_core/satellite/gnss_satellite_types.h"

typedef struct GnssConstellation GnssConstellation;

// Allocation
GnssConstellation *gnss_constellation_create();
// Deallocation
void gnss_constellation_free(GnssConstellation *constellation);

// Notify the constellation of new ephemeris
void gnss_constellation_notify_ephemeris(GnssConstellation *constellation, SatelliteId satellite_id, GnssGpsEphemeris ephemeris);

// Get a pointer to a satellite within the constellation
// Returns:
//      Pointer to the satellite (never NULL, satellite might not have valid ephemeris)
Satellite *gnss_constellation_get_satellite(GnssConstellation *constellation, SatelliteId satellite_id);