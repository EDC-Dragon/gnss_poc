#pragma once

#include "gnss_types.h"
#include "receiver/gnss_receiver.h"

typedef struct
{
    GnssPositionClockState true_receiver;
    Vec3 velocity;
} GnssSimulatedReceiver;

void gnss_simulated_receiver_init(GnssSimulatedReceiver *receiver, Vec3 initial_position, double initial_clock_bias_m, Vec3 initial_velocity);

void gnss_simulated_evaluate_measurement(GnssSimulatedReceiver *simulated_receiver, Satellite *satellites, size_t satellite_count, double reception_time_s, GnssReceiver *receiver);
