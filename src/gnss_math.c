#include "gnss_math.h"

#include <stdlib.h>
#include <math.h>

#include "gnss_types.h"

double gaussian_noise(double sigma)
{
    double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);

    return sigma * sqrt(-2.0 * log(u1)) * cos(2.0 * acos(-1.0) * u2);
}

GnssEstimatedMeasurement compute_pseudorange(
    const Satellite *sat,
    const GnssReceiverState *receiver)
{
    if (!sat || !receiver)
        return (GnssEstimatedMeasurement){.status = GNSS_INVALID_ARGUMENT};

    double distance = vec3_distance(&sat->position, &receiver->position);

    double evaluated_pseudorange = distance + receiver->clock_bias_m;

    if (!isfinite(evaluated_pseudorange))
        return (GnssEstimatedMeasurement){.status = GNSS_NUMERICAL_ERROR};

    return (GnssEstimatedMeasurement){.status = GNSS_OK, .pseudorange = evaluated_pseudorange};
}

double compute_residue(
    double measured_pseudorange,
    double estimated_pseudorange)
{
    return measured_pseudorange - estimated_pseudorange;
}