#include "physical_types.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

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

GnssResult compute_pseudorange(
    const Satellite *sat,
    const Receiver *receiver)
{
    if (!sat || !receiver)
        return (GnssResult){.status = GNSS_INVALID_ARGUMENT};

    double distance = vec3_distance(&sat->position, &receiver->position);

    double evaluated_pseudorange = distance + receiver->clock_bias_m;

    if (!isfinite(evaluated_pseudorange))
        return (GnssResult){.status = GNSS_NUMERICAL_ERROR};

    return (GnssResult){.status = GNSS_OK, .pseudorange = evaluated_pseudorange};
}

void compute_pseudoranges(
    const Satellite *satellites,
    size_t satellite_count,
    const Receiver *receiver,
    GnssResult *results)
{
    if (!satellites || !receiver || !results || satellite_count <= 0)
        return;

    for (size_t satellite_index = 0; satellite_index < satellite_count; satellite_index++)
    {
        const Satellite *satellite = satellites + satellite_index;
        GnssResult *result = results + satellite_index;
        *result = compute_pseudorange(satellite, receiver);
    }
}