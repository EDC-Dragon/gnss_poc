#include "gnss_solver.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>

#include "gnss_types.h"
#include "gnss_simulation.h"
#include "gnss_math.h"
#include "gnss_receiver.h"

GnssSolver *gnss_solver_create()
{
    GnssSolver *solver = malloc(sizeof(GnssSolver));
    return solver;
}

void gnss_solver_free(GnssSolver *solver)
{
    free(solver);
}

void gnss_solver_initialize(GnssSolver *solver)
{
    solver->estimated_receiver = (GnssReceiverState){
        .position = {0.0, 0.0, 0.0},
        .clock_bias_m = 0.0};
}

void gnss_solver_print_estimated_receiver(GnssSolver *solver)
{
    printf("Estimated GnssReceiverState Position: (%f, %f, %f)\n", solver->estimated_receiver.position.x, solver->estimated_receiver.position.y, solver->estimated_receiver.position.z);
    printf("Estimated GnssReceiverState Clock Bias: %f\n", solver->estimated_receiver.clock_bias_m);
}

void gnss_solver_update(GnssSolver *solver, GnssSatelliteData *satellitesData, size_t satellite_count)
{
    for (size_t i = 0; i < satellite_count; i++)
    {
        GnssEstimatedMeasurement result = compute_pseudorange(satellitesData[i].measurement.satellite_position, &solver->estimated_receiver);
        if (result.status == GNSS_OK)
            satellitesData[i].predicted_pseudorange = result.pseudorange;
        else
            satellitesData[i].predicted_pseudorange = NAN;
    }

    for (size_t i = 0; i < satellite_count; i++)
    {
        if (satellitesData[i].measurement.status == GNSS_OK)
        {
            double residue = compute_residue(satellitesData[i].measurement.pseudorange, satellitesData[i].predicted_pseudorange);

            Vec3 satellite_to_estimated_receiver = vec3_subtract(&solver->estimated_receiver.position, &satellitesData[i].measurement.satellite_position);
            vec3_normalize(&satellite_to_estimated_receiver);
            satellitesData[i].observation.jacobian[0] = satellite_to_estimated_receiver.x;
            satellitesData[i].observation.jacobian[1] = satellite_to_estimated_receiver.y;
            satellitesData[i].observation.jacobian[2] = satellite_to_estimated_receiver.z;
            satellitesData[i].observation.jacobian[3] = 1.0;
            satellitesData[i].observation.residue = residue;
        }
    }
}

void gnss_solver_build_system(GnssSolver *solver, GnssSatelliteData *satellitesData, size_t satellite_count)
{
    solver->system = (GnssSystem){0};
    for (size_t sat = 0; sat < satellite_count; sat++)
    {
        double sigma_m = satellitesData[sat].measurement.sigma_m;
        double weight = 0.0;
        if (isfinite(sigma_m) && sigma_m > 0.0)
            weight = 1.0 / (sigma_m * sigma_m);
        for (size_t i = 0; i < 4; i++)
            for (size_t j = 0; j < 4; j++)
                solver->system.A[i][j] += weight * satellitesData[sat].observation.jacobian[i] * satellitesData[sat].observation.jacobian[j];
        for (size_t i = 0; i < 4; i++)
            solver->system.B[i] += weight * satellitesData[sat].observation.jacobian[i] * satellitesData[sat].observation.residue;
    }
}

void gnss_solver_print_system(GnssSolver *solver)
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

void gnss_solver_solve_system(GnssSolver *solver)
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
            return;
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
}

void gnss_solver_apply_system_correction(GnssSolver *solver)
{
    solver->estimated_receiver.position.x += solver->system.state_correction[0];
    solver->estimated_receiver.position.y += solver->system.state_correction[1];
    solver->estimated_receiver.position.z += solver->system.state_correction[2];
    solver->estimated_receiver.clock_bias_m += solver->system.state_correction[3];
}

void gnss_solver_solve(GnssSolver *solver, int max_iterations, double convergence_threshold, GnssSatelliteData *satellitesData, size_t satellite_count, bool verbose)
{
    for (int i = 0; i < max_iterations; i++)
    {
        if (verbose)
        {
            printf("Iteration %d\n", i);
        }
        gnss_solver_update(solver, satellitesData, satellite_count);
        gnss_solver_build_system(solver, satellitesData, satellite_count);
        gnss_solver_solve_system(solver);
        gnss_solver_apply_system_correction(solver);
        if (verbose)
        {
            gnss_solver_print_estimated_receiver(solver);
        }
        double sum = 0.0;
        for (int a = 0; a < 4; a++)
        {
            sum += fabs(solver->system.state_correction[a]);
        }
        if (sum < convergence_threshold)
        {
            break;
        }
    }
}