#pragma once

#include "physical_types.h"

typedef struct
{
    Vec3 jacobian;
    double residue;
} GnssObservation;

typedef struct
{
    Satellite *satellite;

    GnssResult measured_pseudorange;
    double predicted_pseudorange;

    GnssObservation observation;
} GnssSatelliteData;

typedef struct
{
    GnssSatelliteData *satellitesData;
    size_t satellite_count;
    Receiver estimated_receiver;
} GnssSolver;

GnssSolver *create_gnss_solver(size_t max_satellite_count);

void initialize_gnss_solver(GnssSolver *solver, Satellite *satellites, size_t satellite_count);

void update_gnss_solver(GnssSolver *solver);

void free_gnss_solver(GnssSolver *solver);