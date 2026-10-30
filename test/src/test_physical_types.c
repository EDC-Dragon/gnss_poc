#include "test_physical_types.h"

#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "gnss_types.h"
#include "receiver/solver/gnss_solver.h"
#include "gnss_math.h"
#include "gnss_simulation.h"
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

void run_basic_gnss_test()
{
    GnssSimulatedReceiver simulated_receiver;
    Vec3 initial_position = {4200000.0, 170000.0, 4780000.0};
    double initial_clock_bias_m = 150.0;
    Vec3 initial_velocity = {100.0, 100.0, 100.0};
    gnss_simulated_receiver_init(&simulated_receiver, initial_position, initial_clock_bias_m, initial_velocity);

    double noise_sigma_ratio = 1.0;
    double bias_ratio = 1.0;

    Satellite satellites[] = {
        {.id = 1, .position = {15600000.0, 7540000.0, 20140000.0}, .error_model = {.bias_m = 2.0, .noise_sigma_m = 2.0}},
        {.id = 2, .position = {18760000.0, 2750000.0, 18610000.0}, .error_model = {.bias_m = 1.0, .noise_sigma_m = 5.0}},
        {.id = 3, .position = {17610000.0, 14630000.0, 13480000.0}, .error_model = {.bias_m = -2.0, .noise_sigma_m = 1.0}},
        {.id = 4, .position = {19170000.0, 610000.0, 18390000.0}, .error_model = {.bias_m = -3.0, .noise_sigma_m = 7.0}},
        {.id = 5, .position = {17800000.0, -13400000.0, 14600000.0}, .error_model = {.bias_m = 2.0, .noise_sigma_m = 8.0}},
        {.id = 6, .position = {-13400000.0, 10200000.0, 21000000.0}, .error_model = {.bias_m = 1.0, .noise_sigma_m = 2.0}}};
    size_t satellite_count = sizeof(satellites) / sizeof(satellites[0]);
    for (size_t i = 0; i < satellite_count; i++)
    {
        satellites[i].error_model.noise_sigma_m *= noise_sigma_ratio;
        satellites[i].error_model.bias_m *= bias_ratio;
    }

    GnssReceiver *receiver = gnss_receiver_create(satellite_count);
    gnss_receiver_initialize(receiver);

    double time_s = 0.0;
    for (int i = 0; i < 1000; i++)
    {
        gnss_simulated_evaluate_measurement(&simulated_receiver, satellites, satellite_count, time_s, receiver);
        gnss_receiver_update(receiver, time_s);
        time_s += 1.0;
    }

    gnss_receiver_free(receiver);
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

void test_physical_types()
{
    // run_pseudorange_tests();
    run_basic_gnss_test();
}