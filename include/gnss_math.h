#pragma once

#include "gnss_types.h"
#include "gnss_satellite.h"
#include "gnss_receiver.h"

double gaussian_noise(double sigma);

double gaussian_noise_at_time(const char *satellite_name, double time_s, double sigma_m);

GnssEstimatedMeasurement compute_pseudorange(
    const Satellite *sat,
    const GnssReceiverState *receiver);

double compute_residue(
    double measured_pseudorange,
    double estimated_pseudorange);