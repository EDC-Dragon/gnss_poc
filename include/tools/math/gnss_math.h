#pragma once

#include "gnss_core/gnss_types.h"
#include "tools/math/gnss_math_ecef.h"

#define C_MS 299792458.0

GnssStatus compute_pseudorange(
    const Vec3 satellite_position,
    const GnssPositionClockState *receiver,
    double *pseudorange_m);
