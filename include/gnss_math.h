#pragma once

#include "gnss_types.h"
#include "gnss_satellite.h"
#include "gnss_receiver.h"

double gaussian_noise(double sigma);

GnssEstimatedMeasurement compute_pseudorange(
    const Satellite *sat,
    const Receiver *receiver);

double compute_residue(
    double measured_pseudorange,
    double estimated_pseudorange);