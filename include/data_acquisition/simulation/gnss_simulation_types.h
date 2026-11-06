#pragma once

#include <stdbool.h>

#include "gnss_core/gnss_types.h"

#define GNSS_SIMULATION_MAX_SATELLITES_PER_MEASUREMENT 32
#define GNSS_SIMULATION_MAX_EPOCHS 10000

// Real receiver state in the simulation
typedef struct
{
    Vec3 position;
    Vec3 velocity;
} GnssSimulationRealReceiverState;

// Single epoch
typedef struct
{
    double time_s;
    GnssMeasurement *measurements[GNSS_SIMULATION_MAX_SATELLITES_PER_MEASUREMENT];
    int measurement_count;
    GnssSimulationRealReceiverState real_receiver_state;
} GnssSimulationEpoch;

// Complete imulation data (mmultiple epochs)
typedef struct
{
    bool is_valid;
    GnssSimulationEpoch *epochs[GNSS_SIMULATION_MAX_EPOCHS];
    int epoch_count;
} GnssSimulationData;
