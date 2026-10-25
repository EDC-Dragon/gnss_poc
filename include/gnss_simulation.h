#pragma once

#include "gnss_types.h"
#include "gnss_solver.h"

typedef struct
{
    Receiver true_receiver;
} GnssSimulatedReceiver;

GnssMeasurement simulate_pseudorange_measurement(
    const Satellite *sat,
    const Receiver *true_receiver,
    double arrival_time_s);

void gnss_simulated_receiver_init(GnssSimulatedReceiver *receiver, Receiver true_receiver);

void gnss_simulated_evaluate_measurement(GnssSimulatedReceiver *simulated_receiver, double time_s, GnssSolver *solver);