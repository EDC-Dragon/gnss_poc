#pragma once

#include "gnss_core/gnss_types.h"
#include "gnss_core/satellite/gnss_satellite.h"
#include "gnss_core/receiver/gnss_receiver.h"

GnssStatus compute_pseudorange(
    const Vec3 satellite_position,
    const GnssPositionClockState *receiver,
    double *pseudorange_m);
