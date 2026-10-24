#include "test_physical_types.h"

#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "physical_types.h"
#include "gnss_solver.h"

typedef struct PseudoRangeTestCase
{
    Satellite sat;
    Receiver receiver;
    GnssResult expected_result;
    bool should_pass;
} PseudoRangeTestCase;

typedef struct MultiPseudoRangeTestCase
{
    Satellite *satellites;
    Receiver receiver;
    GnssResult *expected_results;
    bool should_pass;
} MultiPseudoRangeTestCase;

bool compare_gnss_results(GnssResult a, GnssResult b)
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
    size_t satellite_count = sizeof(satellites) / sizeof(satellites[0]);

    Receiver receivers[] = {
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 0.0},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 0.0},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = -50.0},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 100.0},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 38.0 - sqrt(2041.0)},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 300.0},
        {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 42.0 - sqrt(140000.0)}};

    GnssResult gnssResult[] = {
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
        GnssResult gnssResult = compute_pseudorange(&test.sat, &test.receiver);
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

    // Compute pseudoranges for all satellites
    Receiver receiver = {
        .position = {10.0, 20.0, 30.0},
        .clock_bias_m = 150.0};

    GnssResult monoReceiverGnssResult[] = {
        {.status = GNSS_OK, .pseudorange = 20140.032516231684},
        {.status = GNSS_OK, .pseudorange = 175.079872407969},
        {.status = GNSS_OK, .pseudorange = 1140.656348084440},
        {.status = GNSS_OK, .pseudorange = 200.089919145473},
        {.status = GNSS_OK, .pseudorange = 157.810249675907},
        {.status = GNSS_OK, .pseudorange = 20000140.000032499433},
        {.status = GNSS_OK, .pseudorange = 486.749164809655}};

    MultiPseudoRangeTestCase multi_test = {
        .satellites = satellites,
        .receiver = receiver,
        .expected_results = monoReceiverGnssResult,
        .should_pass = true};
    GnssResult results[sizeof(satellites) / sizeof(satellites[0])];
    compute_pseudoranges(satellites, satellite_count, &receiver, results);

    printf("========================================\n");
    printf("Running multi-satellite pseudorange tests...\n");
    for (size_t i = 0; i < satellite_count; i++)
    {
        bool result = compare_gnss_results(results[i], multi_test.expected_results[i]);
        if (result == multi_test.should_pass)
        {
            printf("Satellite %lu pseudorange subtest PASSED\n", i);
        }
        else
        {
            printf("Satellite %lu pseudorange subtest FAILED\n", i);
            printf("Expected: status=%d, pseudorange=%f\n", multi_test.expected_results[i].status, multi_test.expected_results[i].pseudorange);
            printf("Got: status=%d, pseudorange=%f\n", results[i].status, results[i].pseudorange);
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

void run_residue_test()
{
    Receiver true_receiver = {
        .position = {10.0, 20.0, 30.0},
        .clock_bias_m = 150.0};

    Satellite satellites[] = {
        {.position = {20000.0, 0.0, 0.0}},
        {.position = {3.0, 4.0, 12.0}},
        {.position = {1000.0, 0.0, 0.0}},
        {.position = {-3.0, -4.0, -12.0}},
        {.position = {13.0, 24.0, 36.0}},
        {.position = {20000000.0, 0.0, 0.0}},
        {.position = {100.0, 200.0, 300.0}}};
    size_t satellite_count = sizeof(satellites) / sizeof(satellites[0]);

    GnssSolver *solver;
    solver = create_gnss_solver(satellite_count);
    initialize_gnss_solver(solver, satellites, satellite_count);

    for (size_t i = 0; i < solver->satellite_count; i++)
    {
        solver->satellitesData[i].measured_pseudorange = compute_pseudorange(solver->satellitesData[i].satellite, &true_receiver);
    }
    update_gnss_solver(solver);

    free_gnss_solver(solver);
}

void test_physical_types()
{
    // run_pseudorange_tests();
    run_residue_test();
}