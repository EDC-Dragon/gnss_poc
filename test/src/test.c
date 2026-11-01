#include <stdio.h>
#include <string.h>

#include "test_gnss_math_matrix.h"
#include "test_gnss_math_gnss.h"
#include "test_gnss_simulation.h"

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Usage: %s <argument>\n", argv[0]);
        return 1;
    }
    else
    {
        printf("Argument: %s\n", argv[1]);
        if (strcmp(argv[1], "math_gnss") == 0)
        {
            test_gnss_math_gnss();
        }
        else if (strcmp(argv[1], "math_matrix") == 0)
        {
            test_gnss_math_matrix();
        }
        else if (strcmp(argv[1], "sim") == 0)
        {
            test_gnss_sim();
        }
    }
    return 0;
}