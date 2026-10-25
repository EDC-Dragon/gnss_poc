#pragma once

#include "gnss_types.h"
#include "gnss_receiver.h"

typedef struct
{
    GnssReceiverState true_receiver;
} GnssSimulatedReceiver;

void gnss_simulated_receiver_init(GnssSimulatedReceiver *receiver, GnssReceiverState true_receiver);

void gnss_simulated_evaluate_measurement(GnssSimulatedReceiver *simulated_receiver, Satellite *satellites, size_t satellite_count, double arrival_time_s, GnssReceiver *receiver);