#pragma once

#include "gnss_types.h"
#include "gnss_satellite.h"
#include "receiver/gnss_receiver.h"

double gaussian_noise(double sigma);

double gaussian_noise_at_time(const uint64_t satellite_id, const double time_s, const double sigma_m);

GnssEstimatedMeasurement compute_pseudorange(
    const Vec3 satellite_position,
    const GnssReceiverState *receiver);

double compute_residue(
    double measurement,
    double estimated_pseudorange);