#include "gnss_receiver.h"

#include <stdlib.h>

GnssReceiver *gnss_receiver_create(size_t max_satellite_count)
{
    GnssReceiver *receiver = (GnssReceiver *)malloc(sizeof(GnssReceiver));
    receiver->satellitesData = (GnssSatelliteData *)malloc(max_satellite_count * sizeof(GnssSatelliteData));
    receiver->satellite_count = 0;
    receiver->max_satellite_count = max_satellite_count;
    receiver->solver = gnss_solver_create();
    return receiver;
}

void gnss_receiver_initialize(GnssReceiver *receiver)
{
    for (size_t i = 0; i < receiver->satellite_count; i++)
    {
        receiver->satellitesData[i].satellite = NULL;
        receiver->satellitesData[i] = (GnssSatelliteData){0};
    }
    gnss_solver_initialize(receiver->solver);
}

void gnss_receiver_free(GnssReceiver *receiver)
{
    free(receiver->satellitesData);
    gnss_solver_free(receiver->solver);
    free(receiver);
}

void gnss_receiver_update_satellite(GnssReceiver *receiver, Satellite *satellite, GnssMeasurement measurement)
{
    if (receiver->satellite_count >= receiver->max_satellite_count)
        return;
    receiver->satellitesData[receiver->satellite_count].satellite = satellite;
    receiver->satellitesData[receiver->satellite_count].measured_pseudorange = measurement;
    receiver->satellite_count++;
}

void gnss_receiver_update(GnssReceiver *receiver)
{
    gnss_solver_solve(receiver->solver, 100, 10e-7, receiver->satellitesData, receiver->satellite_count, false);
    gnss_solver_print_estimated_receiver(receiver->solver);
    receiver->satellite_count = 0;
}
