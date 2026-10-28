#pragma once

#include <stddef.h>
#include <stdbool.h>

#include "gnss_types.h"
#include "gnss_satellite.h"

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

typedef struct
{
    GnssSystem system;

    GnssReceiverState estimated_receiver;
} GnssSolver;

GnssSolver *gnss_solver_create();

// Initialize the GNSS solver with the given satellites and satellite count.
void gnss_solver_initialize(GnssSolver *solver);

// Solve the GNSS system iteratively until convergence or the maximum number of iterations is reached.
GnssStatus gnss_solver_solve(GnssSolver *solver, int max_iterations, double convergence_threshold, GnssSatelliteData *satellitesData, size_t satellite_count, bool verbose);

void gnss_solver_print_estimated_receiver(GnssSolver *solver);

// Free the resources allocated for the GNSS solver.
void gnss_solver_free(GnssSolver *solver);