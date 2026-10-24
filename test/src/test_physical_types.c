#include "test_physical_types.h"

#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "physical_types.h"

struct PseudoRangeTestCase
{
    Satellite sat;
    Receiver receiver;
    GnssResult expected_result;
    bool should_pass;
};

bool run_pseudorange_test(const struct PseudoRangeTestCase *test)
{
    GnssResult gnssResult = compute_pseudorange(&test->sat, &test->receiver);
    bool result = gnssResult.status == test->expected_result.status && gnssResult.pseudorange == test->expected_result.pseudorange;
    return result == test->should_pass;
}

void run_pseudorange_tests()
{
    // Tests cases, generated with AI
    struct PseudoRangeTestCase tests[] = {
        // 1. Erroneous to check test // Distance simple + biais positif
        {.sat = {.position = {20000.0, 0.0, 0.0}}, .receiver = {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 150.0}, .expected_result = {.status = GNSS_OK, .pseudorange = 0.0}, .should_pass = false},

        // 2. Distance simple + biais positif
        {.sat = {.position = {20000.0, 0.0, 0.0}}, .receiver = {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 150.0}, .expected_result = {.status = GNSS_OK, .pseudorange = 20150.0}, .should_pass = true},

        // 3. Distance nulle, biais nul
        {.sat = {.position = {0.0, 0.0, 0.0}}, .receiver = {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 0.0}, .expected_result = {.status = GNSS_OK, .pseudorange = 0.0}, .should_pass = true},

        // 4. Distance 3D (3, 4, 12)
        {.sat = {.position = {3.0, 4.0, 12.0}}, .receiver = {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 0.0}, .expected_result = {.status = GNSS_OK, .pseudorange = 13.0}, .should_pass = true},

        // 5. Biais négatif
        {.sat = {.position = {1000.0, 0.0, 0.0}}, .receiver = {.position = {0.0, 0.0, 0.0}, .clock_bias_m = -50.0}, .expected_result = {.status = GNSS_OK, .pseudorange = 950.0}, .should_pass = true},

        // 6. Coordonnées négatives
        {.sat = {.position = {-3.0, -4.0, -12.0}}, .receiver = {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 100.0}, .expected_result = {.status = GNSS_OK, .pseudorange = 113.0}, .should_pass = true},

        // 7. Récepteur décalé de l'origine
        {.sat = {.position = {13.0, 24.0, 36.0}}, .receiver = {.position = {10.0, 20.0, 24.0}, .clock_bias_m = 25.0}, .expected_result = {.status = GNSS_OK, .pseudorange = 38.0}, .should_pass = true},

        // 8. Géométrie GNSS simplifiée (~20 000 km)
        {.sat = {.position = {20000000.0, 0.0, 0.0}}, .receiver = {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 300.0}, .expected_result = {.status = GNSS_OK, .pseudorange = 20000300.0}, .should_pass = true},

        // 9. Distance nulle avec biais non nul
        {.sat = {.position = {100.0, 200.0, 300.0}}, .receiver = {.position = {100.0, 200.0, 300.0}, .clock_bias_m = 42.0}, .expected_result = {.status = GNSS_OK, .pseudorange = 42.0}, .should_pass = true}};

    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); i++)
    {
        printf("Test %lu %s\n", i + 1, run_pseudorange_test(&tests[i]) ? "PASSED" : "FAILED");
    }

    Satellite satellites[] = {
        {.position = {20000.0, 0.0, 0.0}},
        {.position = {3.0, 4.0, 12.0}},
        {.position = {1000.0, 0.0, 0.0}},
        {.position = {-3.0, -4.0, -12.0}},
        {.position = {13.0, 24.0, 36.0}},
        {.position = {20000000.0, 0.0, 0.0}},
        {.position = {100.0, 200.0, 300.0}}};

    size_t satellite_count = sizeof(satellites) / sizeof(satellites[0]);

    Receiver receiver = {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 0.0};
    GnssResult results[sizeof(satellites) / sizeof(satellites[0])];
    compute_pseudoranges(satellites, satellite_count, &receiver, results);
    for (size_t i = 0; i < satellite_count; i++)
    {
        printf("Satellite %lu pseudorange: %f\n", i + 1, results[i].pseudorange);
    }
}

void test_physical_types()
{
    run_pseudorange_tests();
}