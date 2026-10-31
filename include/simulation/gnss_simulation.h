#pragma once

#include "gnss_types.h"
#include "receiver/gnss_receiver.h"
#include "simulation/gnss_simulation_types.h"

void gnss_simulated_receiver_init(GnssSimulatedReceiver *receiver, Vec3 initial_position, double initial_clock_bias_m, Vec3 initial_velocity);

void gnss_simulated_evaluate_measurement(GnssSimulatedReceiver *simulated_receiver, Satellite *satellites, size_t satellite_count, double reception_time_s, GnssReceiver *receiver);

typedef struct
{
    GnssSimulationData *data;
    int current_epoch;
} GnssSimulator;

GnssSimulator *gnss_simulator_create();
void gnss_simulator_free(GnssSimulator *sim);

void gnss_simulator_initialize(GnssSimulator *sim, const char *path);

GnssSimulationEpoch *gnss_simulator_step(GnssSimulator *sim);