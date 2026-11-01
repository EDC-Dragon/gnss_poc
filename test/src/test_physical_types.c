#include "test_physical_types.h"

#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "gnss_types.h"
#include "receiver/solver/gnss_solver.h"
#include "gnss_math.h"
#include "simulation/gnss_simulation.h"
#include "simulation/gnss_simulation_csv_reader.h"
#include "receiver/gnss_receiver.h"

typedef struct PseudoRangeTestCase
{
    Satellite sat;
    GnssPositionClockState receiver;
    GnssStatus expected_status;
    double expected_pseudorange_m;
    bool should_pass;
} PseudoRangeTestCase;

typedef struct MultiPseudoRangeTestCase
{
    Satellite *satellites;
    GnssPositionClockState receiver;
    GnssStatus *expected_statuses;
    double *expected_pseudoranges_m;
    bool should_pass;
} MultiPseudoRangeTestCase;

void run_pseudorange_tests()
{
    bool all_tests_passed = true;

    // Tests data, generated with AI
    Satellite satellites[] = {
        {.position = {20000.0, 0.0, 0.0}},
        {.position = {3.0, 4.0, 12.0}},
        {.position = {1000.0, 0.0, 0.0}},
        {.position = {-3.0, -4.0, -12.0}},
        {.position = {13.0, 24.0, 36.0}},
        {.position = {20000000.0, 0.0, 0.0}},
        {.position = {100.0, 200.0, 300.0}}};

    GnssPositionClockState receivers[] = {
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 0.0},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 0.0},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = -50.0},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 100.0},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 38.0 - sqrt(2041.0)},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 300.0},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 42.0 - sqrt(140000.0)}};

    GnssStatus gnssResult[] = {
        GNSS_OK,
        GNSS_OK,
        GNSS_OK,
        GNSS_OK,
        GNSS_OK,
        GNSS_OK,
        GNSS_OK};

    double expected_pseudoranges_m[] = {
        20000.0,
        13.0,
        950.0,
        113.0,
        38.0,
        20000300.0,
        42.0};

    // Satellite pseudorange tests
    PseudoRangeTestCase tests[] = {
        {.sat = satellites[0], .receiver = receivers[0], .expected_status = gnssResult[0], .expected_pseudorange_m = expected_pseudoranges_m[0], .should_pass = true},
        {.sat = satellites[0], .receiver = receivers[0], .expected_status = gnssResult[1], .expected_pseudorange_m = expected_pseudoranges_m[1], .should_pass = false},
        {.sat = satellites[1], .receiver = receivers[1], .expected_status = gnssResult[1], .expected_pseudorange_m = expected_pseudoranges_m[2], .should_pass = true},
        {.sat = satellites[2], .receiver = receivers[2], .expected_status = gnssResult[2], .expected_pseudorange_m = expected_pseudoranges_m[3], .should_pass = true},
        {.sat = satellites[3], .receiver = receivers[3], .expected_status = gnssResult[3], .expected_pseudorange_m = expected_pseudoranges_m[4], .should_pass = true},
        {.sat = satellites[4], .receiver = receivers[4], .expected_status = gnssResult[4], .expected_pseudorange_m = expected_pseudoranges_m[5], .should_pass = true},
        {.sat = satellites[5], .receiver = receivers[5], .expected_status = gnssResult[5], .expected_pseudorange_m = expected_pseudoranges_m[6], .should_pass = true},
        {.sat = satellites[6], .receiver = receivers[6], .expected_status = gnssResult[6], .expected_pseudorange_m = expected_pseudoranges_m[7], .should_pass = true}};

    printf("========================================\n");
    printf("Running pseudorange tests...\n");
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++)
    {
        PseudoRangeTestCase test = tests[i];
        double pseudorange_m;
        GnssStatus status = compute_pseudorange(test.sat.position, &test.receiver, &pseudorange_m);
        bool result = true;
        if (status != test.expected_status)
            result = false;
        if (pseudorange_m != test.expected_pseudorange_m)
            result = false;
        if (result == test.should_pass)
        {
            printf("Test %lu PASSED\n", i);
        }
        else
        {
            printf("Test %lu FAILED\n", i);
            printf("Expected: status=%d, pseudorange=%f\n", test.expected_status, test.expected_pseudorange_m);
            printf("Got: status=%d, pseudorange=%f\n", status, pseudorange_m);
            all_tests_passed = false;
        }
    }

    printf("========================================\n");

    if (all_tests_passed)
    {
        printf("All pseudorange tests PASSED\n");
    }
    else
    {
        printf("Some pseudorange tests FAILED\n");
    }
}

/*
bool is_symmetric = true;
for (int i = 0; i < 4; i++)
{
    for (int j = i + 1; j < 4; j++)
    {
        if (solver->system.A[i][j] != solver->system.A[j][i])
        {
            printf("System A[%d][%d] is NOT symmetric: %f vs %f\n", i, j, solver->system.A[i][j], solver->system.A[j][i]);
            is_symmetric = false;
        }
    }
}
if (!is_symmetric)
{
    printf("System A is NOT symmetric\n");
}

*/

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

void test_physical_types()
{
    // run_pseudorange_tests();
    // run_basic_gnss_test();
    run_sim_csv_test("/home/edwindc/workspace/gnss_poc/simulation_data/batch_2026-10-31_20-29-50/");
}