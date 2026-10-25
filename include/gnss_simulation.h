#pragma once

#include "physical_types.h"
#include "gnss_solver.h"

typedef struct
{
    Receiver true_receiver;
} GnssSimulatedReceiver;

void gnss_simulated_receiver_init(GnssSimulatedReceiver *receiver, Receiver true_receiver);

void gnss_simulated_evaluate_measurement(GnssSimulatedReceiver *simulated_receiver, double time_s, GnssSolver *solver);