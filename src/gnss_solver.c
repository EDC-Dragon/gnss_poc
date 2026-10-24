#include "gnss_solver.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#include "physical_types.h"

GnssSolver *create_gnss_solver(size_t max_satellite_count)
{
    GnssSolver *solver = malloc(sizeof(GnssSolver));
    solver->satellitesData = malloc(sizeof(GnssSatelliteData) * max_satellite_count);
    return solver;
}

void free_gnss_solver(GnssSolver *solver)
{
    free(solver->satellitesData);
    free(solver);
}

void initialize_gnss_solver(GnssSolver *solver, Satellite *satellites, size_t satellite_count)
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

void update_gnss_solver(GnssSolver *solver)
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
            printf("Residue: %f\n", residue);

            Vec3 satellite_to_estimated_receiver = vec3_subtract(&solver->estimated_receiver.position, &solver->satellitesData[i].satellite->position);
            double distance = vec3_length(&satellite_to_estimated_receiver);
            printf("Distance to estimated receiver: %f\n", distance);
            printf("Satellite %lu to receiver vector: (%f, %f, %f)\n", i, satellite_to_estimated_receiver.x, satellite_to_estimated_receiver.y, satellite_to_estimated_receiver.z);
            vec3_normalize(&satellite_to_estimated_receiver);
            printf("Normalized satellite to estimated receiver vector: (%f, %f, %f)\n", satellite_to_estimated_receiver.x, satellite_to_estimated_receiver.y, satellite_to_estimated_receiver.z);
            solver->satellitesData[i].observation.jacobian = satellite_to_estimated_receiver;
            solver->satellitesData[i].observation.residue = residue;
        }
    }
}