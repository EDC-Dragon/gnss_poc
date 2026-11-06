#pragma once

#include "gnss_core/gnss_types.h"
#include "gnss_core/satellite/gnss_satellite.h"
#include "gnss_core/receiver/estimator/gnss_state_estimator.h"
#include "gnss_core/receiver/solver/gnss_solver.h"

// GNSS receiver representation, receives GNSS measurements and estimates receiver state
typedef struct GnssReceiver GnssReceiver;

// Allocation/Deallocation
GnssReceiver *gnss_receiver_create(size_t max_satellite_count);
void gnss_receiver_free(GnssReceiver *receiver);

// Initialization
void gnss_receiver_initialize(GnssReceiver *receiver);

// Update the receiver with new measurements
// Returns:
//      GNSS_STATUS_OK if the update was successful
//      GNSS_STATUS_X otherwise
GnssStatus gnss_receiver_update(GnssReceiver *receiver, GnssMeasurement *measurements, size_t measurement_count, double time_s);

// Get the current estimated state of the receiver
// Returns:
//      Pointer to the estimated state, might not be fully initialized
GnssEstimatedState *gnss_receiver_get_estimated_state(GnssReceiver *receiver);