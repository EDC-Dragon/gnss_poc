#include "test_physical_types.h"

#include <stdbool.h>
#include <stdio.h>

#include "physical_types.h"

struct TestCase
{
    Satellite sat;
    Receiver receiver;
    double expected_pseudorange;
    bool should_pass;
};

bool run_test_case(const struct TestCase *test)
{
    double pseudorange = compute_pseudorange(&test->sat, &test->receiver);
    bool result = pseudorange == test->expected_pseudorange;
    return result == test->should_pass;
}

void test_physical_types()
{
    // Tests cases, generated with AI
    struct TestCase tests[] = {
        // 1. Erroneous to check test // Distance simple + biais positif
        {.sat = {.position = {20000.0, 0.0, 0.0}}, .receiver = {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 150.0}, .expected_pseudorange = 0.0, .should_pass = false},

        // 2. Distance simple + biais positif
        {.sat = {.position = {20000.0, 0.0, 0.0}}, .receiver = {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 150.0}, .expected_pseudorange = 20150.0, .should_pass = true},

        // 3. Distance nulle, biais nul
        {.sat = {.position = {0.0, 0.0, 0.0}}, .receiver = {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 0.0}, .expected_pseudorange = 0.0, .should_pass = true},

        // 4. Distance 3D (3, 4, 12)
        {.sat = {.position = {3.0, 4.0, 12.0}}, .receiver = {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 0.0}, .expected_pseudorange = 13.0, .should_pass = true},

        // 5. Biais négatif
        {.sat = {.position = {1000.0, 0.0, 0.0}}, .receiver = {.position = {0.0, 0.0, 0.0}, .clock_bias_m = -50.0}, .expected_pseudorange = 950.0, .should_pass = true},

        // 6. Coordonnées négatives
        {.sat = {.position = {-3.0, -4.0, -12.0}}, .receiver = {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 100.0}, .expected_pseudorange = 113.0, .should_pass = true},

        // 7. Récepteur décalé de l'origine
        {.sat = {.position = {13.0, 24.0, 36.0}}, .receiver = {.position = {10.0, 20.0, 24.0}, .clock_bias_m = 25.0}, .expected_pseudorange = 38.0, .should_pass = true},

        // 8. Géométrie GNSS simplifiée (~20 000 km)
        {.sat = {.position = {20000000.0, 0.0, 0.0}}, .receiver = {.position = {0.0, 0.0, 0.0}, .clock_bias_m = 300.0}, .expected_pseudorange = 20000300.0, .should_pass = true},

        // 9. Distance nulle avec biais non nul
        {.sat = {.position = {100.0, 200.0, 300.0}}, .receiver = {.position = {100.0, 200.0, 300.0}, .clock_bias_m = 42.0}, .expected_pseudorange = 42.0, .should_pass = true}};

    for (int i = 0; i < sizeof(tests) / sizeof(tests[0]); i++)
    {
        printf("Test %d %s\n", i + 1, run_test_case(&tests[i]) ? "PASSED" : "FAILED");
    }
}