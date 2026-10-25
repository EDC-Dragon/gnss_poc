#include "gnss_math.h"

#include <stdlib.h>
#include <math.h>

double gaussian_noise(double sigma)
{
    double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);

    return sigma * sqrt(-2.0 * log(u1)) * cos(2.0 * acos(-1.0) * u2);
}