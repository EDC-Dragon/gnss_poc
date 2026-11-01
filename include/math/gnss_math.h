#pragma once

#include "gnss_types.h"
#include "satellite/gnss_satellite.h"
#include "receiver/gnss_receiver.h"

GnssStatus compute_pseudorange(
    const Vec3 satellite_position,
    const GnssPositionClockState *receiver,
    double *pseudorange_m);
