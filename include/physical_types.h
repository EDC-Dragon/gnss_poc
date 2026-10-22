#pragma once

#define C_MS 299792458.0

typedef struct
{
    double x;
    double y;
    double z;
} Vec3;

Vec3 vec3_subtract(const Vec3 *a, const Vec3 *b);
double vec3_length(const Vec3 *v);

typedef struct
{
    Vec3 position;
    double clock_bias_m;
} Receiver;

typedef struct
{
    Vec3 position;
} Satellite;

double compute_pseudorange(
    const Satellite *sat,
    const Receiver *receiver);
