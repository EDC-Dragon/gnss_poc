#include "gnss_math.h"

#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include "gnss_types.h"

double gaussian_noise(double sigma)
{
    double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0);

    return sigma * sqrt(-2.0 * log(u1)) * cos(2.0 * acos(-1.0) * u2);
}

#define GNSS_PI 3.14159265358979323846

// Mélange déterministe de 64 bits (SplitMix64)
static uint64_t mix64(uint64_t x)
{
    x += UINT64_C(0x9E3779B97F4A7C15);
    x = (x ^ (x >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
    x = (x ^ (x >> 27)) * UINT64_C(0x94D049BB133111EB);
    return x ^ (x >> 31);
}

// Convertit 53 bits aléatoires en un double dans ]0, 1[
static double uniform01(uint64_t x)
{
    return ((double)(x >> 11) + 0.5) / 9007199254740992.0;
}

uint64_t satellite_id_from_name(const char *name)
{
    uint64_t hash = UINT64_C(14695981039346656037);

    while (*name)
    {
        hash ^= (unsigned char)*name++;
        hash *= UINT64_C(1099511628211);
    }

    return hash;
}

// Gaussian noise at a given time for a satellite (AI Generated)
double gaussian_noise_at_time(
    const char *satellite_name,
    double time_s,
    double sigma_m)
{
    // Une valeur différente par milliseconde
    int64_t epoch_ms = (int64_t)llround(time_s * 1000.0);

    // Graine déterministe satellite + époque
    uint64_t seed = mix64(
        satellite_id_from_name(satellite_name) ^ mix64((uint64_t)epoch_ms) ^ UINT64_C(0x123456789ABCDEF0));

    // Deux uniformes indépendantes
    double u1 = uniform01(mix64(seed));
    double u2 = uniform01(mix64(seed + 1));

    // Transformation de Box-Muller
    double z = sqrt(-2.0 * log(u1)) * cos(2.0 * GNSS_PI * u2);

    return sigma_m * z;
}

GnssEstimatedMeasurement compute_pseudorange(
    const Satellite *sat,
    const GnssReceiverState *receiver)
{
    if (!sat || !receiver)
        return (GnssEstimatedMeasurement){.status = GNSS_INVALID_ARGUMENT};

    double distance = vec3_distance(&sat->position, &receiver->position);

    double evaluated_pseudorange = distance + receiver->clock_bias_m;

    if (!isfinite(evaluated_pseudorange))
        return (GnssEstimatedMeasurement){.status = GNSS_NUMERICAL_ERROR};

    return (GnssEstimatedMeasurement){.status = GNSS_OK, .pseudorange = evaluated_pseudorange};
}

double compute_residue(
    double measured_pseudorange,
    double estimated_pseudorange)
{
    return measured_pseudorange - estimated_pseudorange;
}