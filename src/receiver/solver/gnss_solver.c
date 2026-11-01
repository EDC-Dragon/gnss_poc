#include "receiver/solver/gnss_solver.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>

#include "gnss_types.h"
#include "simulation/gnss_simulation.h"
#include "math/gnss_math_noise.h"
#include "receiver/gnss_receiver.h"

#include "gnss_solver_internal.h"

struct GnssSolver
{
    GnssSolverInternal *internal;

    GnssPositionClockState estimated_state;
};

GnssSolver *gnss_solver_create(size_t max_satellite_count)
{
    GnssSolver *solver = malloc(sizeof(GnssSolver));
    if (!solver)
        return NULL;
    solver->internal = gnss_solver_internal_create(max_satellite_count);
    if (!solver->internal)
    {
        free(solver);
        return NULL;
    }
    return solver;
}

void gnss_solver_free(GnssSolver *solver)
{
    gnss_solver_internal_free(solver->internal);
    free(solver);
}

void gnss_solver_print(GnssSolver *solver)
{
    printf("Estimated GnssPositionClockState Position: (%f, %f, %f)\n", solver->estimated_state.position.x, solver->estimated_state.position.y, solver->estimated_state.position.z);
    printf("Estimated GnssPositionClockState Clock Bias: %f\n", solver->estimated_state.clock_bias_m);
}

GnssPositionClockState *gnss_solver_get_estimated_state(GnssSolver *solver)
{
    return &solver->estimated_state;
}

void gnss_solver_initialize(GnssSolver *solver)
{
    solver->estimated_state = (GnssPositionClockState){
        .position = {0.0, 0.0, 0.0},
        .clock_bias_m = 0.0};
}

GnssStatus gnss_solver_solve(GnssSolver *solver, const GnssMeasurement *measurements, size_t measurement_count, double epoch_time_s)
{
    return gnss_solver_internal_solve(solver->internal, &solver->estimated_state, measurements, measurement_count, epoch_time_s, 100, 1e-7, false);
}