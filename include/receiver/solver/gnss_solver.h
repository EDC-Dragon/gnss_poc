#pragma once

#include <stddef.h>
#include <stdbool.h>

#include "gnss_types.h"
#include "gnss_satellite.h"

#include "gnss_solver_types.h"

typedef struct GnssSolver GnssSolver;

GnssSolver *
gnss_solver_create(size_t max_satellite_count);

// Initialize the GNSS solver.
void gnss_solver_initialize(GnssSolver *solver);

// Solve the GNSS system iteratively until convergence or the maximum number of iterations is reached.
GnssStatus gnss_solver_solve(GnssSolver *solver, double epoch_time_s, int max_iterations, double convergence_threshold, const GnssMeasurement *measurements, size_t satellite_count, bool verbose);

void gnss_solver_print_estimated_receiver(GnssSolver *solver);

// Get the estimated GNSS state from the solver.
GnssPositionClockState *gnss_solver_get_estimated_state(GnssSolver *solver);

// Free the resources allocated for the GNSS solver.
void gnss_solver_free(GnssSolver *solver);