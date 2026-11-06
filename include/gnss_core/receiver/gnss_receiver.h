#pragma once

#include "gnss_core/gnss_types.h"
#include "gnss_core/satellite/gnss_satellite.h"
#include "gnss_core/receiver/estimator/gnss_state_estimator.h"
#include "gnss_core/receiver/solver/gnss_solver.h"

typedef struct GnssReceiver GnssReceiver;

// Allocation/Deallocation
GnssReceiver *gnss_receiver_create(size_t max_satellite_count);
void gnss_receiver_free(GnssReceiver *receiver);

// Initialization
void gnss_receiver_initialize(GnssReceiver *receiver);

void gnss_receiver_update(GnssReceiver *receiver, GnssMeasurement *measurements, size_t measurement_count, double time_s);
