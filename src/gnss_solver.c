#include "gnss_solver.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#include "physical_types.h"

GnssSolver *gnss_solver_create(size_t max_satellite_count)
{
    GnssSolver *solver = malloc(sizeof(GnssSolver));
    solver->satellitesData = malloc(sizeof(GnssSatelliteData) * max_satellite_count);
    return solver;
}

void gnss_solver_free(GnssSolver *solver)
{
    free(solver->satellitesData);
    free(solver);
}

void gnss_solver_initialize(GnssSolver *solver, Satellite *satellites, size_t satellite_count)
{
    solver->satellite_count = satellite_count;
    solver->estimated_receiver = (Receiver){
        .position = {0.0, 0.0, 0.0},
        .clock_bias_m = 0.0};
    for (size_t i = 0; i < satellite_count; i++)
    {
        solver->satellitesData[i].satellite = &satellites[i];
        solver->satellitesData[i].predicted_pseudorange = NAN;
        solver->satellitesData[i].measured_pseudorange = (GnssResult){.status = GNSS_NUMERICAL_ERROR, .pseudorange = NAN};
        solver->satellitesData[i].observation = (GnssObservation){.jacobian = {0.0, 0.0, 0.0}, .residue = NAN};
    }
}

void gnss_solver_print_estimated_receiver(GnssSolver *solver)
{
    printf("Estimated Receiver Position: (%f, %f, %f)\n", solver->estimated_receiver.position.x, solver->estimated_receiver.position.y, solver->estimated_receiver.position.z);
    printf("Estimated Receiver Clock Bias: %f\n", solver->estimated_receiver.clock_bias_m);
}

void gnss_solver_update(GnssSolver *solver)
{
    for (size_t i = 0; i < solver->satellite_count; i++)
    {
        solver->satellitesData[i].satellite = solver->satellitesData[i].satellite;
        GnssResult result = compute_pseudorange(solver->satellitesData[i].satellite, &solver->estimated_receiver);
        if (result.status == GNSS_OK)
            solver->satellitesData[i].predicted_pseudorange = result.pseudorange;
        else
            solver->satellitesData[i].predicted_pseudorange = NAN;
    }

    for (size_t i = 0; i < solver->satellite_count; i++)
    {
        if (solver->satellitesData[i].measured_pseudorange.status == GNSS_OK)
        {
            double residue = compute_residue(solver->satellitesData[i].measured_pseudorange.pseudorange, solver->satellitesData[i].predicted_pseudorange);

            Vec3 satellite_to_estimated_receiver = vec3_subtract(&solver->estimated_receiver.position, &solver->satellitesData[i].satellite->position);
            double distance = vec3_length(&satellite_to_estimated_receiver);
            vec3_normalize(&satellite_to_estimated_receiver);
            solver->satellitesData[i].observation.jacobian[0] = satellite_to_estimated_receiver.x;
            solver->satellitesData[i].observation.jacobian[1] = satellite_to_estimated_receiver.y;
            solver->satellitesData[i].observation.jacobian[2] = satellite_to_estimated_receiver.z;
            solver->satellitesData[i].observation.jacobian[3] = 1.0;
            solver->satellitesData[i].observation.residue = residue;
        }
    }
}

void gnss_solver_build_system(GnssSolver *solver)
{
    solver->system = (GnssSystem){0};
    for (size_t sat = 0; sat < solver->satellite_count; sat++)
    {
        for (size_t i = 0; i < 4; i++)
            for (size_t j = 0; j < 4; j++)
                solver->system.A[i][j] += solver->satellitesData[sat].observation.jacobian[i] * solver->satellitesData[sat].observation.jacobian[j];
        for (size_t i = 0; i < 4; i++)
            solver->system.B[i] += solver->satellitesData[sat].observation.jacobian[i] * solver->satellitesData[sat].observation.residue;
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

void gnss_solver_iterate(GnssSolver *solver, int max_iterations, bool verbose)
{
    for (int i = 0; i < max_iterations; i++)
    {
        if (verbose)
        {
            printf("Iteration %d\n", i);
        }
        gnss_solver_update(solver);
        gnss_solver_build_system(solver);
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
        if (sum < 1e-12)
        {
            break;
        }
    }
}