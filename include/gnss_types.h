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
double vec3_distance(const Vec3 *a, const Vec3 *b);

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
    double sigma_m;
    double transmission_time_s;
    double arrival_time_s;
} GnssMeasurement;

typedef struct
{
    GnssStatus status;
    double pseudorange;
} GnssEstimatedMeasurement;
