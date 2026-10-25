#include "physical_types.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "gnss_math.h"

Vec3 vec3_subtract(const Vec3 *a, const Vec3 *b)
{
    Vec3 result;
    result.x = a->x - b->x;
    result.y = a->y - b->y;
    result.z = a->z - b->z;
    return result;
}

double vec3_length(const Vec3 *v)
{
    return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

double vec3_distance(const Vec3 *a, const Vec3 *b)
{
    Vec3 diff = vec3_subtract(a, b);
    return vec3_length(&diff);
}

void vec3_normalize(Vec3 *v)
{
    double length = vec3_length(v);
    if (length > 0)
    {
        v->x /= length;
        v->y /= length;
        v->z /= length;
    }
}

GnssMeasurement simulate_pseudorange_measurement(
    const Satellite *sat,
    const Receiver *true_receiver,
    double arrival_time_s)
{
    if (!sat || !true_receiver)
        return (GnssMeasurement){.status = GNSS_INVALID_ARGUMENT};

    double distance = vec3_distance(&sat->position, &true_receiver->position);

    double pseudorange = distance + true_receiver->clock_bias_m;

    if (!isfinite(pseudorange))
        return (GnssMeasurement){.status = GNSS_NUMERICAL_ERROR};

    double noise = sat->error_model.bias_m + gaussian_noise(sat->error_model.noise_sigma_m);
    double pseudorange_with_noise = pseudorange + noise;
    double sigma_m = sat->error_model.noise_sigma_m;

    double transmission_time_s = sat->time_s - distance / C_MS;

    return (GnssMeasurement){.status = GNSS_OK, .pseudorange = pseudorange_with_noise, .sigma_m = sigma_m, .transmission_time_s = transmission_time_s, .arrival_time_s = arrival_time_s};
}

GnssEstimatedMeasurement compute_pseudorange(
    const Satellite *sat,
    const Receiver *receiver)
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