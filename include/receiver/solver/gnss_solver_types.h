#pragma once

#include "gnss_types.h"

typedef struct
{
    double jacobian[4];
    double residue;
} GnssObservation;

typedef struct
{
    GnssMeasurement measurement;

    double predicted_pseudorange;

    GnssObservation observation;
} GnssSatelliteData;

typedef struct
{
    double A[4][4];
    double B[4];
    double state_correction[4];
} GnssSystem;
