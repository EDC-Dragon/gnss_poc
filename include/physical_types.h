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
    double bias_m;
    double noise_sigma_m;
} SatelliteErrorModel;

typedef struct
{
    Vec3 position;
    double time_s;
    SatelliteErrorModel error_model;
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
    double sigma_m;
    double transmission_time_s;
    double arrival_time_s;
} GnssMeasurement;

GnssMeasurement simulate_pseudorange_measurement(
    const Satellite *sat,
    const Receiver *true_receiver,
    double arrival_time_s);

typedef struct
{
    GnssStatus status;
    double pseudorange;
} GnssEstimatedMeasurement;

GnssEstimatedMeasurement compute_pseudorange(
    const Satellite *sat,
    const Receiver *receiver);

double compute_residue(
    double measured_pseudorange,
    double estimated_pseudorange);