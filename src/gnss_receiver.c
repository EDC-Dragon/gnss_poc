#include "gnss_receiver.h"

#include <stdlib.h>
#include <string.h>

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

typedef struct
{
    bool found;
    size_t index;
} GnssSatelliteSearchResult;

GnssSatelliteSearchResult gnss_receiver_find_satellite(GnssReceiver *receiver, Satellite *sat)
{
    for (size_t i = 0; i < receiver->satellite_count; i++)
        if (strcmp(receiver->satellitesData[i].satellite->name, sat->name) == 0)
            return (GnssSatelliteSearchResult){true, i};
    return (GnssSatelliteSearchResult){false, 0};
}

void gnss_receiver_link_satellite(GnssReceiver *receiver, Satellite *satellite)
{
    GnssSatelliteSearchResult res = gnss_receiver_find_satellite(receiver, satellite);
    if (!res.found && receiver->satellite_count < receiver->max_satellite_count)
    {
        receiver->satellitesData[receiver->satellite_count].satellite = satellite;
        receiver->satellitesData[receiver->satellite_count].measured_pseudorange = (GnssMeasurement){.status = GNSS_INVALID_ARGUMENT};
        receiver->satellite_count++;
    }
}

void gnss_receiver_unlink_satellite(GnssReceiver *receiver, Satellite *satellite)
{
    GnssSatelliteSearchResult res = gnss_receiver_find_satellite(receiver, satellite);
    if (res.found)
    {
        receiver->satellite_count--;
        for (size_t i = res.index; i < receiver->satellite_count; i++)
            receiver->satellitesData[i] = receiver->satellitesData[i + 1];
    }
}

void gnss_receiver_unlink_obsolete_satellites(GnssReceiver *receiver, double time_s, double satellite_obsolescence_age_s)
{
    // Reverse order to avoid shifting issues
    for (size_t i = receiver->satellite_count; i > 0; i--)
    {
        double age = time_s - receiver->satellitesData[i - 1].measured_pseudorange.arrival_time_s;
        if (age > satellite_obsolescence_age_s)
            gnss_receiver_unlink_satellite(receiver, receiver->satellitesData[i - 1].satellite);
    }
}

void gnss_receiver_update_satellite(GnssReceiver *receiver, Satellite *satellite, GnssMeasurement measurement)
{
    GnssSatelliteSearchResult res = gnss_receiver_find_satellite(receiver, satellite);
    if (!res.found)
    {
        printf("Satellite %s not found, linking it\n", satellite->name);
        gnss_receiver_link_satellite(receiver, satellite);
    }
    res = gnss_receiver_find_satellite(receiver, satellite);
    if (res.found)
        receiver->satellitesData[res.index].measured_pseudorange = measurement;
}

void gnss_receiver_update(GnssReceiver *receiver, double time_s)
{
    double satellite_obsolescence_age_s = 10.0;
    gnss_receiver_unlink_obsolete_satellites(receiver, time_s, satellite_obsolescence_age_s);

    double obsolescence_age_s = 0.2;
    GnssSatelliteData valid_satellite_data[receiver->satellite_count];
    size_t valid_satellite_count = 0;

    for (size_t i = 0; i < receiver->satellite_count; i++)
    {
        if (receiver->satellitesData[i].measured_pseudorange.status != GNSS_OK)
            continue;
        double data_age_s = time_s - receiver->satellitesData[i].measured_pseudorange.arrival_time_s;
        if (obsolescence_age_s < data_age_s)
            continue;
        valid_satellite_data[valid_satellite_count] = receiver->satellitesData[i];
        valid_satellite_count++;
    }

    if (valid_satellite_count < 4)
    {
        printf("Not enough valid satellites\n");
        return;
    }

    gnss_solver_solve(receiver->solver, 100, 10e-7, valid_satellite_data, valid_satellite_count, false);

    gnss_solver_print_estimated_receiver(receiver->solver);
}
