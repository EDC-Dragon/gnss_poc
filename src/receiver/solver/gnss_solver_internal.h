#pragma once

#include "gnss_types.h"
#include "receiver/solver/gnss_solver.h"

typedef struct GnssSolverInternal GnssSolverInternal;

GnssSolverInternal *gnss_solver_internal_create(size_t max_satellite_count);
void gnss_solver_internal_free(GnssSolverInternal *solver);

GnssStatus gnss_solver_internal_solve(GnssSolverInternal *solver, GnssPositionClockState *estimated_state, const GnssMeasurement *measurements, size_t measurement_count, double epoch_time_s, int max_iterations, double convergence_threshold, bool verbose);