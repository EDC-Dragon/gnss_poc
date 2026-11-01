#include "test_gnss_simulation.h"

#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "gnss_types.h"
#include "receiver/solver/gnss_solver.h"
#include "math/gnss_math.h"
#include "simulation/gnss_simulation.h"
#include "simulation/gnss_simulation_csv_reader.h"
#include "receiver/gnss_receiver.h"

void run_sim_csv_test(const char *path)
{
    GnssSimulator *simulator = gnss_simulator_create();
    gnss_simulator_initialize(simulator, path);
    printf("Starting simulation\n");

    GnssReceiver *receiver = gnss_receiver_create(GNSS_SIMULATION_MAX_SATELLITES_PER_MEASUREMENT);
    gnss_receiver_initialize(receiver);

    GnssSimulationEpoch *epoch = gnss_simulator_step(simulator);
    while (epoch)
    {
        printf("Epoch time: %f\n", epoch->time_s);

        // Convert into usable array
        GnssMeasurement measurements[epoch->measurement_count];
        for (int i = 0; i < epoch->measurement_count; i++)
            measurements[i] = *epoch->measurements[i];
        gnss_receiver_update(receiver, measurements, epoch->measurement_count, epoch->time_s);

        epoch = gnss_simulator_step(simulator);
    }
    printf("Simulation done\n");

    gnss_simulator_free(simulator);
}

void test_gnss_sim()
{
    run_sim_csv_test("/home/edwindc/workspace/gnss_poc/simulation_data/batch_2026-10-31_20-29-50/");
}