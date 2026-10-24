#pragma once

#include <stddef.h>

#define C_MS 299792458.0

typedef struct
{
    double x;
    double y;
    double z;
} Vec3;

Vec3 vec3_subtract(const Vec3 *a, const Vec3 *b);
double vec3_length(const Vec3 *v);
void vec3_normalize(Vec3 *v);

typedef struct
{
    Vec3 position;
    double clock_bias_m;
} Receiver;

typedef struct
{
    Vec3 position;
} Satellite;

typedef enum
{
    GNSS_OK,
    GNSS_INVALID_ARGUMENT,
    GNSS_NUMERICAL_ERROR
} GnssStatus;

typedef struct
{
    GnssStatus status;
    double pseudorange;
} GnssResult;

GnssResult compute_pseudorange(
    const Satellite *sat,
    const Receiver *receiver);

void compute_pseudoranges(
    const Satellite *satellites,
    size_t satellite_count,
    const Receiver *receiver,
    GnssResult *results);
