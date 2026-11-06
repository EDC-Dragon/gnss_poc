#pragma once

#include <stddef.h>
#include <stdbool.h>

#include "gnss_core/gnss_types.h"
#include "gnss_core/satellite/gnss_satellite.h"

typedef struct GnssSolver GnssSolver;

///////////////////////
// GENERAL FUNCTIONS //

// Create a GNSS solver instance.
// max satellite count: maximal allowed number of satellites to track
GnssSolver *gnss_solver_create(size_t max_satellite_count);

// Free the resources allocated for the GNSS solver.
void gnss_solver_free(GnssSolver *solver);

// Print the estimated receiver state
void gnss_solver_print(GnssSolver *solver);

//////////////////////
// SOLVER FUNCTIONS //

// Initialize the solver state to zero
void gnss_solver_initialize(GnssSolver *solver);

// Solve the GNSS system iteratively until convergence or the maximum number of iterations is reached.
GnssStatus gnss_solver_solve(GnssSolver *solver, const GnssMeasurement *measurements, size_t measurement_count, double epoch_time_s);

// Get the estimated GNSS state from the solver.
GnssPositionClockState *gnss_solver_get_estimated_state(GnssSolver *solver);
