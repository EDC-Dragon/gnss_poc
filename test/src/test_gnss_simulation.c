#include "test_gnss_simulation.h"

#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "gnss_core/gnss_types.h"
#include "gnss_core/receiver/solver/gnss_solver.h"
#include "tools/math/gnss_math.h"
#include "data_acquisition/simulation/gnss_simulation.h"
#include "data_acquisition/simulation/gnss_simulation_csv_reader.h"
#include "gnss_core/receiver/gnss_receiver.h"

bool run_sim_csv_test(const char *path)
{
    GnssSimulator *simulator = gnss_simulator_create();
    gnss_simulator_initialize(simulator, path);
    printf("Starting simulation\n");

    GnssReceiver *receiver = gnss_receiver_create(GNSS_SIMULATION_MAX_SATELLITES_PER_MEASUREMENT);
    gnss_receiver_initialize(receiver);

    double last_error_norm = 0.0;
    GnssSimulationEpoch *epoch = gnss_simulator_step(simulator);
    while (epoch)
    {
        printf("Epoch time: %f\n", epoch->time_s);

        // Convert into usable array
        GnssMeasurement measurements[epoch->measurement_count];
        for (int i = 0; i < epoch->measurement_count; i++)
            measurements[i] = *epoch->measurements[i];
        if (gnss_receiver_update(receiver, measurements, epoch->measurement_count, epoch->time_s) != GNSS_OK)
        {
            printf("Receiver update failed\n");
        }
        else
        {
            printf("Receiver update succeeded\n");
            GnssEstimatedState *estimated_state = gnss_receiver_get_estimated_state(receiver);
            if (estimated_state)
            {
                printf("Estimated position: (%f, %f, %f)\n",
                       estimated_state->position.x,
                       estimated_state->position.y,
                       estimated_state->position.z);
                Vec3 real_position = epoch->real_receiver_state.position;
                Vec3 diff = vec3_subtract(&estimated_state->position, &real_position);
                double error_norm = vec3_length(&diff);
                printf("Error norm: %f\n", error_norm);
                last_error_norm = error_norm;
            }
        }

        epoch = gnss_simulator_step(simulator);
    }
    printf("Simulation done\n");

    gnss_simulator_free(simulator);

    printf("Last error norm: %f\n", last_error_norm);
    if (25.0 < last_error_norm)
    {
        printf("Warning: Last error norm exceeded 25.0\n");
        return false;
    }
    else
    {
        printf("Last error norm is within acceptable range\n");
        return true;
    }
}

void test_gnss_sim()
{
    if (!run_sim_csv_test("/home/edwindc/workspace/gnss_poc/simulation_data/batch_2026-10-31_20-29-50/"))
        printf("Simulation test failed\n");
    else
        printf("Simulation test succeeded\n");
}