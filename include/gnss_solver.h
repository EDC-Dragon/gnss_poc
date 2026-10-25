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
    GnssSystem system;

    GnssReceiverState estimated_receiver;
} GnssSolver;

GnssSolver *gnss_solver_create();

// Initialize the GNSS solver with the given satellites and satellite count.
void gnss_solver_initialize(GnssSolver *solver);

// Update the GNSS solver with the latest measurements and predictions.
void gnss_solver_update(GnssSolver *solver, GnssSatelliteData *satellitesData, size_t satellite_count);

// Build the system of equations for the GNSS solver.
void gnss_solver_build_system(GnssSolver *solver, GnssSatelliteData *satellitesData, size_t satellite_count);

// Print the current system of equations for the GNSS solver.
void gnss_solver_print_system(GnssSolver *solver);

// Solve the system of equations for the GNSS solver.
void gnss_solver_solve_system(GnssSolver *solver);

// Apply the computed correction to the GNSS solver's state.
void gnss_solver_apply_system_correction(GnssSolver *solver);

// Print the estimated receiver state.
void gnss_solver_print_estimated_receiver(GnssSolver *solver);

// Solve the GNSS system iteratively until convergence or the maximum number of iterations is reached.
void gnss_solver_solve(GnssSolver *solver, int max_iterations, double convergence_threshold, GnssSatelliteData *satellitesData, size_t satellite_count, bool verbose);

// Free the resources allocated for the GNSS solver.
void gnss_solver_free(GnssSolver *solver);