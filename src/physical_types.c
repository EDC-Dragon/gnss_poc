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

double compute_pseudorange(
    const Satellite *sat,
    const Receiver *receiver)
{
    double distance = vec3_distance(&sat->position, &receiver->position);

    double pseudorange = distance + receiver->clock_bias_m;

    return pseudorange;
}
