#pragma once

#include "gnss_core/gnss_types.h"
#include "gnss_core/receiver/gnss_receiver.h"
#include "data_acquisition/simulation/gnss_simulation_types.h"

typedef struct
{
    GnssSimulationData *data;
    int current_epoch;
} GnssSimulator;

GnssSimulator *gnss_simulator_create();
void gnss_simulator_free(GnssSimulator *sim);

void gnss_simulator_initialize(GnssSimulator *sim, const char *path);

GnssSimulationEpoch *gnss_simulator_step(GnssSimulator *sim);