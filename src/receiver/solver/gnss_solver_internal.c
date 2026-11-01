#include "gnss_solver_internal.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>

#include "gnss_types.h"
#include "simulation/gnss_simulation.h"
#include "gnss_math.h"
#include "receiver/gnss_receiver.h"

typedef struct
{
    double jacobian[4];
    double residue;
} GnssObservation;

typedef struct
{
    double A[4][4];
    double B[4];
    double state_correction[4];
} GnssSystem;

typedef struct
{
    double predicted_pseudorange;
    double sigma_m;

    GnssObservation observation;
} GnssSolverSatelliteContext;

struct GnssSolverInternal
{
    size_t max_satellite_count;
    GnssSolverSatelliteContext *satellite_contexts;

    GnssSystem system;

    GnssPositionClockState transient_state;
};

GnssSolverInternal *gnss_solver_internal_create(size_t max_satellite_count)
{
    GnssSolverInternal *solver = malloc(sizeof(GnssSolverInternal));
    solver->max_satellite_count = max_satellite_count;
    solver->satellite_contexts = calloc(max_satellite_count, sizeof(GnssSolverSatelliteContext));
    return solver;
}

void gnss_solver_internal_free(GnssSolverInternal *solver)
{
    free(solver->satellite_contexts);
    free(solver);
}

void gnss_solver_internal_update_observations(GnssSolverInternal *solver, const GnssMeasurement *measurements, size_t satellite_count)
{
    for (size_t i = 0; i < satellite_count; i++)
    {
        double pseudorange_m;
        solver->satellite_contexts[i].sigma_m = measurements[i].sigma_m;
        GnssStatus status = compute_pseudorange(measurements[i].satellite_position, &solver->transient_state, &pseudorange_m);
        if (status == GNSS_OK)
            solver->satellite_contexts[i].predicted_pseudorange = pseudorange_m;
        else
            solver->satellite_contexts[i].predicted_pseudorange = NAN;
    }

    for (size_t i = 0; i < satellite_count; i++)
    {
        if (measurements[i].status == GNSS_OK)
        {
            double residue = compute_residue(measurements[i].pseudorange_m, solver->satellite_contexts[i].predicted_pseudorange);

            Vec3 satellite_to_estimated_receiver = vec3_subtract(&solver->transient_state.position, &measurements[i].satellite_position);
            vec3_normalize(&satellite_to_estimated_receiver);
            solver->satellite_contexts[i].observation.jacobian[0] = satellite_to_estimated_receiver.x;
            solver->satellite_contexts[i].observation.jacobian[1] = satellite_to_estimated_receiver.y;
            solver->satellite_contexts[i].observation.jacobian[2] = satellite_to_estimated_receiver.z;
            solver->satellite_contexts[i].observation.jacobian[3] = 1.0;
            solver->satellite_contexts[i].observation.residue = residue;
        }
    }
}

void gnss_solver_internal_build_system(GnssSolverInternal *solver, size_t satellite_count)
{
    solver->system = (GnssSystem){0};
    for (size_t sat = 0; sat < satellite_count; sat++)
    {
        double sigma_m = solver->satellite_contexts[sat].sigma_m;
        double weight = 0.0;
        if (isfinite(sigma_m) && sigma_m > 0.0)
            weight = 1.0 / (sigma_m * sigma_m);
        for (size_t i = 0; i < 4; i++)
            for (size_t j = 0; j < 4; j++)
                solver->system.A[i][j] += weight * solver->satellite_contexts[sat].observation.jacobian[i] * solver->satellite_contexts[sat].observation.jacobian[j];
        for (size_t i = 0; i < 4; i++)
            solver->system.B[i] += weight * solver->satellite_contexts[sat].observation.jacobian[i] * solver->satellite_contexts[sat].observation.residue;
    }
}

void gnss_solver_internal_print_system(GnssSolverInternal *solver)
{
    printf("System A:\n");
    for (size_t i = 0; i < 4; i++)
    {
        for (size_t j = 0; j < 4; j++)
        {
            printf("%f ", solver->system.A[i][j]);
        }
        printf("\n");
    }
    printf("System B:\n");
    for (size_t i = 0; i < 4; i++)
    {
        printf("%f\n", solver->system.B[i]);
    }
}

GnssStatus gnss_solver_internal_solve_system(GnssSolverInternal *solver)
{
    for (size_t root_index = 0; root_index < 3; root_index++)
    {
        size_t largest_row = root_index;
        double largest_value = fabs(solver->system.A[root_index][root_index]);
        for (size_t ii = root_index + 1; ii < 4; ii++)
        {
            if (largest_value < fabs(solver->system.A[ii][root_index]))
            {
                largest_value = fabs(solver->system.A[ii][root_index]);
                largest_row = ii;
            }
        }
        if (largest_value < 1e-12)
        {
            printf("Numerical error: largest pivot is too small\n");
            return GNSS_NUMERICAL_ERROR;
        }
        if (largest_row != root_index)
        {
            // Swap
            for (size_t j = 0; j < 4; j++)
            {
                double temp = solver->system.A[root_index][j];
                solver->system.A[root_index][j] = solver->system.A[largest_row][j];
                solver->system.A[largest_row][j] = temp;
            }
            double temp = solver->system.B[root_index];
            solver->system.B[root_index] = solver->system.B[largest_row];
            solver->system.B[largest_row] = temp;
        }
        // Largest row is now at root_index, proceed with elimination

        for (size_t i = root_index + 1; i < 4; i++)
        {
            double factor = solver->system.A[i][root_index] / solver->system.A[root_index][root_index];
            for (size_t j = root_index; j < 4; j++)
            {
                solver->system.A[i][j] -= factor * solver->system.A[root_index][j];
            }
            solver->system.B[i] -= factor * solver->system.B[root_index];
        }
    }

    for (int i = 3; i >= 0; i--)
    {
        double result = solver->system.B[i];
        for (int j = i + 1; j < 4; j++)
        {
            result -= solver->system.A[i][j] * solver->system.state_correction[j];
        }
        solver->system.state_correction[i] = result / solver->system.A[i][i];
    }
    return GNSS_OK;
}

void gnss_solver_internal_apply_system_correction(GnssSolverInternal *solver)
{
    solver->transient_state.position.x += solver->system.state_correction[0];
    solver->transient_state.position.y += solver->system.state_correction[1];
    solver->transient_state.position.z += solver->system.state_correction[2];
    solver->transient_state.clock_bias_m += solver->system.state_correction[3];
}

GnssStatus gnss_solver_internal_solve(GnssSolverInternal *solver, GnssPositionClockState *estimated_state, const GnssMeasurement *measurements, size_t measurement_count, double epoch_time_s, int max_iterations, double convergence_threshold, bool verbose)
{
    if (measurement_count > solver->max_satellite_count)
        return GNSS_INVALID_ARGUMENT;

    solver->transient_state = *estimated_state;
    solver->transient_state.time_s = epoch_time_s;

    bool converged = false;
    for (int i = 0; i < max_iterations; i++)
    {
        if (verbose)
        {
            printf("Iteration %d\n", i);
        }
        gnss_solver_internal_update_observations(solver, measurements, measurement_count);
        gnss_solver_internal_build_system(solver, measurement_count);
        switch (gnss_solver_internal_solve_system(solver))
        {
        case GNSS_OK:
            break;
        case GNSS_NUMERICAL_ERROR:
            return GNSS_NUMERICAL_ERROR;
        case GNSS_INVALID_ARGUMENT:
            return GNSS_INVALID_ARGUMENT;
        } // TODO: Transient swapable structures to prevent partially editing data
        gnss_solver_internal_apply_system_correction(solver);
        double sum = 0.0;
        for (int a = 0; a < 4; a++)
        {
            sum += fabs(solver->system.state_correction[a]);
        }
        if (sum < convergence_threshold)
        {
            converged = true;
            break;
        }
    }
    if (converged)
    {
        gnss_solver_internal_update_observations(solver, measurements, measurement_count);
        gnss_solver_internal_build_system(solver, measurement_count);

        double inverse_A[4][4];

        GnssStatus status = matrix_inverse(4, solver->system.A, inverse_A);
        if (status != GNSS_OK)
            return status;

        status = matrix_square_submatrix(4, inverse_A, 3, solver->transient_state.covariance);
        if (status != GNSS_OK)
            return status;

        *estimated_state = solver->transient_state;

        return GNSS_OK;
    }
    else
        return GNSS_NUMERICAL_ERROR;
}