#pragma once

#include "gnss_types.h"
#include "gnss_satellite.h"
#include "receiver/estimator/gnss_state_estimator.h"
#include "receiver/solver/gnss_solver.h"

typedef struct
{
    GnssPositionClockState estimated_receiver;

    GnssSolver *solver;

    GnssStateEstimator *state_estimator;

    GnssSatelliteData *satellitesData;
    size_t satellite_count;
    size_t max_satellite_count;
} GnssReceiver;

GnssReceiver *gnss_receiver_create(size_t max_satellite_count);

void gnss_receiver_initialize(GnssReceiver *receiver);

void gnss_receiver_add_measurement(GnssReceiver *receiver, GnssMeasurement measurement);

void gnss_receiver_update(GnssReceiver *receiver, double time);

void gnss_receiver_free(GnssReceiver *receiver);
