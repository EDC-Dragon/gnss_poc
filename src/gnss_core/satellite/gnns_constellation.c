#include "gnss_core/satellite/gnns_constellation.h"

#include <stdlib.h>
#include <string.h>

GnssConstellation *gnss_constellation_create()
{
    GnssConstellation *constellation = (GnssConstellation *)malloc(sizeof(GnssConstellation));
    if (constellation == NULL)
    {
        return NULL;
    }
    memset(constellation->satellites, 0, sizeof(constellation->satellites));
    for (int i = 0; i < MAX_SATELLITES_PER_CONSTELLATION; i++)
    {
        constellation->satellites[i].id = i;
        constellation->satellites[i].has_ephemeris = false;
    }
    return constellation;
}
void gnss_constellation_free(GnssConstellation *constellation)
{
    if (constellation != NULL)
    {
        free(constellation);
    }
}

void gnss_constellation_notify_ephemeris(GnssConstellation *constellation, SatelliteId satellite_id, GnssGpsEphemeris ephemeris)
{
    if (constellation == NULL)
        return;
    constellation->satellites[satellite_id].has_ephemeris = true;
    // Copy ephemeris data to the satellite if needed
    constellation->satellites[satellite_id].ephemeris = ephemeris;
}

Satellite *gnss_constellation_get_satellite(GnssConstellation *constellation, SatelliteId satellite_id)
{
    if (constellation == NULL)
        return NULL;
    return &constellation->satellites[satellite_id];
}