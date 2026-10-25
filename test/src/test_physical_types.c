#include "test_physical_types.h"

#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "gnss_types.h"
#include "gnss_solver.h"
#include "gnss_math.h"
#include "gnss_simulation.h"
#include "gnss_receiver.h"

typedef struct PseudoRangeTestCase
{
    Satellite sat;
    Receiver receiver;
    GnssEstimatedMeasurement expected_result;
    bool should_pass;
} PseudoRangeTestCase;

typedef struct MultiPseudoRangeTestCase
{
    Satellite *satellites;
    Receiver receiver;
    GnssEstimatedMeasurement *expected_results;
    bool should_pass;
} MultiPseudoRangeTestCase;

bool compare_gnss_results(GnssEstimatedMeasurement a, GnssEstimatedMeasurement b)
{
    return a.status == b.status && fabs(a.pseudorange - b.pseudorange) < 1e-6;
}

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

    Receiver receivers[] = {
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 0.0},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 0.0},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = -50.0},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 100.0},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 38.0 - sqrt(2041.0)},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 300.0},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 42.0 - sqrt(140000.0)}};

    GnssEstimatedMeasurement gnssResult[] = {
        {.status = GNSS_OK, .pseudorange = 20000.0},
        {.status = GNSS_OK, .pseudorange = 13.0},
        {.status = GNSS_OK, .pseudorange = 950.0},
        {.status = GNSS_OK, .pseudorange = 113.0},
        {.status = GNSS_OK, .pseudorange = 38.0},
        {.status = GNSS_OK, .pseudorange = 20000300.0},
        {.status = GNSS_OK, .pseudorange = 42.0}};

    // Satellite pseudorange tests
    PseudoRangeTestCase tests[] = {
        {.sat = satellites[0], .receiver = receivers[0], .expected_result = gnssResult[0], .should_pass = true},
        {.sat = satellites[0], .receiver = receivers[0], .expected_result = gnssResult[1], .should_pass = false},
        {.sat = satellites[1], .receiver = receivers[1], .expected_result = gnssResult[1], .should_pass = true},
        {.sat = satellites[2], .receiver = receivers[2], .expected_result = gnssResult[2], .should_pass = true},
        {.sat = satellites[3], .receiver = receivers[3], .expected_result = gnssResult[3], .should_pass = true},
        {.sat = satellites[4], .receiver = receivers[4], .expected_result = gnssResult[4], .should_pass = true},
        {.sat = satellites[5], .receiver = receivers[5], .expected_result = gnssResult[5], .should_pass = true},
        {.sat = satellites[6], .receiver = receivers[6], .expected_result = gnssResult[6], .should_pass = true}};

    printf("========================================\n");
    printf("Running pseudorange tests...\n");
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++)
    {
        PseudoRangeTestCase test = tests[i];
        GnssEstimatedMeasurement gnssResult = compute_pseudorange(&test.sat, &test.receiver);
        bool result = compare_gnss_results(gnssResult, test.expected_result);
        if (result == test.should_pass)
        {
            printf("Test %lu PASSED\n", i);
        }
        else
        {
            printf("Test %lu FAILED\n", i);
            printf("Expected: status=%d, pseudorange=%f\n", test.expected_result.status, test.expected_result.pseudorange);
            printf("Got: status=%d, pseudorange=%f\n", gnssResult.status, gnssResult.pseudorange);
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
    Receiver true_receiver = {
        .position = {4200000.0, 170000.0, 4780000.0},
        .clock_bias_m = 150.0};
    gnss_simulated_receiver_init(&simulated_receiver, true_receiver);

    double noise_sigma_ratio = 1.0;
    double bias_ratio = 1.0;

    Satellite satellites[] = {
        {.position = {15600000.0, 7540000.0, 20140000.0}, .error_model = {.bias_m = 2.0, .noise_sigma_m = 2.0}},
        {.position = {18760000.0, 2750000.0, 18610000.0}, .error_model = {.bias_m = 1.0, .noise_sigma_m = 5.0}},
        {.position = {17610000.0, 14630000.0, 13480000.0}, .error_model = {.bias_m = -2.0, .noise_sigma_m = 1.0}},
        {.position = {19170000.0, 610000.0, 18390000.0}, .error_model = {.bias_m = -3.0, .noise_sigma_m = 7.0}},
        {.position = {17800000.0, -13400000.0, 14600000.0}, .error_model = {.bias_m = 2.0, .noise_sigma_m = 8.0}},
        {.position = {-13400000.0, 10200000.0, 21000000.0}, .error_model = {.bias_m = 1.0, .noise_sigma_m = 2.0}}};
    size_t satellite_count = sizeof(satellites) / sizeof(satellites[0]);
    for (size_t i = 0; i < satellite_count; i++)
    {
        satellites[i].error_model.noise_sigma_m *= noise_sigma_ratio;
        satellites[i].error_model.bias_m *= bias_ratio;
    }

    GnssSolver *solver;
    solver = gnss_solver_create(satellite_count);
    gnss_solver_initialize(solver, satellites, satellite_count);

    double time_s = 0.0;

    gnss_simulated_evaluate_measurement(&simulated_receiver, time_s, solver);

    gnss_solver_solve(solver, 100, 1e-7, false);

    gnss_solver_print_estimated_receiver(solver);

    gnss_solver_free(solver);
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