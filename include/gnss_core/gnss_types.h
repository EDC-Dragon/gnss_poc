#pragma once

#include <stdint.h>
#include "tools/math/gnss_math_ecef.h"

typedef uint64_t SatelliteId;

typedef enum
{
    GNSS_OK,
    GNSS_INVALID_ARGUMENT,
    GNSS_NUMERICAL_ERROR
} GnssStatus;

typedef struct
{
    GnssStatus status;

    double pseudorange_m;
    double sigma_m;

    SatelliteId satellite_id;
    Vec3 satellite_position;

    double transmission_time_s;
    double reception_time_s;
    double availability_time_s;
} GnssMeasurement;

// Physical Time State (Position, Clock Bias and Time)
typedef struct
{
    double time_s;
    Vec3 position;
    double covariance[3][3];
    double clock_bias_m;
} GnssPositionClockState;

// Physical Time State (Position, Clock Bias, Velocity and Time)
typedef struct
{
    double time_s;
    Vec3 position;
    double clock_bias_m;
    Vec3 velocity;
} GnssPositionVelocityTimeState;
