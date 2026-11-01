#include <stdio.h>
#include <string.h>

#include "test_physical_types.h"
#include "test_gnss_math_matrix.h"

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
        if (strcmp(argv[1], "physical_types") == 0)
        {
            test_physical_types();
        }
        else if (strcmp(argv[1], "math_matrix") == 0)
        {
            test_matrix_maths();
        }
    }
    return 0;
}