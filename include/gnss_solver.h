#pragma once

#include <stddef.h>
#include <stdbool.h>

#include "physical_types.h"

typedef struct
{
    double jacobian[4];
    double residue;
} GnssObservation;

typedef struct
{
    Satellite *satellite;

    GnssMeasurement measured_pseudorange;
    double predicted_pseudorange;

    GnssObservation observation;
} GnssSatelliteData;

typedef struct
{
    double A[4][4];
    double B[4];
    double state_correction[4];
} GnssSystem;

typedef struct
{
    GnssSatelliteData *satellitesData;
    size_t satellite_count;

    GnssSystem system;

    Receiver estimated_receiver;
} GnssSolver;

GnssSolver *gnss_solver_create(size_t max_satellite_count);

void gnss_solver_initialize(GnssSolver *solver, Satellite *satellites, size_t satellite_count);

void gnss_solver_update(GnssSolver *solver);

void gnss_solver_build_system(GnssSolver *solver);

void gnss_solver_print_system(GnssSolver *solver);

void gnss_solver_solve_system(GnssSolver *solver);

void gnss_solver_apply_system_correction(GnssSolver *solver);

void gnss_solver_print_estimated_receiver(GnssSolver *solver);

void gnss_solver_solve(GnssSolver *solver, int max_iterations, double convergence_threshold, bool verbose);

void gnss_solver_free(GnssSolver *solver);