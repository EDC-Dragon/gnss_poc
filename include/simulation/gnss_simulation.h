#pragma once

#include "gnss_types.h"
#include "receiver/gnss_receiver.h"
#include "simulation/gnss_simulation_types.h"

typedef struct
{
    GnssSimulationData *data;
    int current_epoch;
} GnssSimulator;

GnssSimulator *gnss_simulator_create();
void gnss_simulator_free(GnssSimulator *sim);

void gnss_simulator_initialize(GnssSimulator *sim, const char *path);

GnssSimulationEpoch *gnss_simulator_step(GnssSimulator *sim);