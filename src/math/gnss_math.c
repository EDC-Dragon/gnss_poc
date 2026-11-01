#include "math/gnss_math.h"

#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include "gnss_types.h"

GnssStatus compute_pseudorange(
    const Vec3 satellite_position,
    const GnssPositionClockState *receiver,
    double *pseudorange_m)
{
    if (!receiver || !pseudorange_m)
        return GNSS_INVALID_ARGUMENT;

    double distance = vec3_distance(&satellite_position, &receiver->position);

    double evaluated_pseudorange = distance + receiver->clock_bias_m;

    if (!isfinite(evaluated_pseudorange))
        return GNSS_NUMERICAL_ERROR;

    *pseudorange_m = evaluated_pseudorange;
    return GNSS_OK;
}