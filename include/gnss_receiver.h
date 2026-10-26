#pragma once

#include "gnss_types.h"
#include "gnss_satellite.h"
#include "gnss_solver.h"

typedef struct
{
    GnssReceiverState estimated_receiver;

    GnssSolver *solver;

    GnssSatelliteData *satellitesData;
    size_t satellite_count;
    size_t max_satellite_count;
} GnssReceiver;

GnssReceiver *gnss_receiver_create(size_t max_satellite_count);

void gnss_receiver_initialize(GnssReceiver *receiver);

void gnss_receiver_link_satellite(GnssReceiver *receiver, Satellite *satellite);

void gnss_receiver_unlink_satellite(GnssReceiver *receiver, Satellite *satellite);

void gnss_receiver_update_satellite(GnssReceiver *receiver, Satellite *satellite, GnssMeasurement measurement);

void gnss_receiver_update(GnssReceiver *receiver, double time);

void gnss_receiver_free(GnssReceiver *receiver);
