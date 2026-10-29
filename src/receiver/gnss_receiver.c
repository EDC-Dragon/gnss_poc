#include "receiver/gnss_receiver.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "receiver/estimator/gnss_state_estimator_types.h"

GnssReceiver *gnss_receiver_create(size_t max_satellite_count)
{
    GnssReceiver *receiver = (GnssReceiver *)malloc(sizeof(GnssReceiver));
    receiver->satellitesData = (GnssSatelliteData *)malloc(max_satellite_count * sizeof(GnssSatelliteData));
    receiver->satellite_count = 0;
    receiver->max_satellite_count = max_satellite_count;
    receiver->solver = gnss_solver_create();
    receiver->state_estimator = gnss_state_estimator_create();
    return receiver;
}

void gnss_receiver_initialize(GnssReceiver *receiver)
{
    for (size_t i = 0; i < receiver->satellite_count; i++)
    {
        receiver->satellitesData[i] = (GnssSatelliteData){0};
    }
    gnss_solver_initialize(receiver->solver);
    gnss_state_estimator_initialize(receiver->state_estimator);
}

void gnss_receiver_free(GnssReceiver *receiver)
{
    free(receiver->satellitesData);
    gnss_solver_free(receiver->solver);
    gnss_state_estimator_free(receiver->state_estimator);
    free(receiver);
}

typedef struct
{
    bool found;
    size_t index;
} GnssSatelliteSearchResult;

GnssSatelliteSearchResult gnss_receiver_find_satellite(GnssReceiver *receiver, SatelliteId satellite_id)
{
    for (size_t i = 0; i < receiver->satellite_count; i++)
        if (receiver->satellitesData[i].measurement.satellite_id == satellite_id)
            return (GnssSatelliteSearchResult){true, i};
    return (GnssSatelliteSearchResult){false, 0};
}

void gnss_receiver_link_satellite(GnssReceiver *receiver, SatelliteId satellite_id, GnssMeasurement measurement)
{
    GnssSatelliteSearchResult res = gnss_receiver_find_satellite(receiver, satellite_id);
    if (!res.found && receiver->satellite_count < receiver->max_satellite_count)
    {
        receiver->satellitesData[receiver->satellite_count].measurement = measurement;
        receiver->satellite_count++;
    }
}

void gnss_receiver_unlink_satellite(GnssReceiver *receiver, SatelliteId satellite_id)
{
    GnssSatelliteSearchResult res = gnss_receiver_find_satellite(receiver, satellite_id);
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
        double age = time_s - receiver->satellitesData[i - 1].measurement.reception_time_s;
        if (age > satellite_obsolescence_age_s)
            gnss_receiver_unlink_satellite(receiver, receiver->satellitesData[i - 1].measurement.satellite_id);
    }
}

void gnss_receiver_add_measurement(GnssReceiver *receiver, GnssMeasurement measurement)
{
    GnssSatelliteSearchResult res = gnss_receiver_find_satellite(receiver, measurement.satellite_id);
    if (!res.found)
    {
        printf("Satellite %lu not found, linking it\n", measurement.satellite_id);
        gnss_receiver_link_satellite(receiver, measurement.satellite_id, measurement);
    }
    res = gnss_receiver_find_satellite(receiver, measurement.satellite_id);
    if (res.found)
    {
        printf("Updating satellite %lu\n", measurement.satellite_id);
        receiver->satellitesData[res.index].measurement = measurement;
    }
}

void gnss_receiver_update(GnssReceiver *receiver, double time_s)
{
    printf("------------------------------------------\n");
    printf("Updating GNSS receiver at time %f\n", time_s);
    double satellite_obsolescence_age_s = 10.0;
    gnss_receiver_unlink_obsolete_satellites(receiver, time_s, satellite_obsolescence_age_s);

    GnssSatelliteData valid_satellite_data[receiver->satellite_count];
    size_t valid_satellite_count = 0;

    double epoch_time_s = 0.0;
    for (size_t i = 0; i < receiver->satellite_count; i++)
    {
        GnssMeasurement *m = &receiver->satellitesData[i].measurement;
        if (m->status != GNSS_OK)
            continue;
        if (m->availability_time_s > time_s)
            continue;
        if (m->reception_time_s > epoch_time_s)
            epoch_time_s = m->reception_time_s;
    }

    printf("Epoch time: %f\n", epoch_time_s);

    for (size_t i = 0; i < receiver->satellite_count; i++)
    {
        GnssMeasurement *m = &receiver->satellitesData[i].measurement;
        if (m->status != GNSS_OK)
            continue;
        if (m->availability_time_s > time_s)
            continue;
        if (fabs(m->reception_time_s - epoch_time_s) > 1e-6)
            continue;
        valid_satellite_data[valid_satellite_count] = receiver->satellitesData[i];
        valid_satellite_count++;
    }

    if (valid_satellite_count < 4)
    {
        printf("Not enough valid satellites\n");
        return;
    }

    printf("Solving...\n");
    gnss_solver_solve(receiver->solver, 100, 10e-7, valid_satellite_data, valid_satellite_count, false);
    gnss_solver_print_estimated_receiver(receiver->solver);

    printf("State estimation...\n");
    gnss_state_estimator_update(receiver->state_estimator, &receiver->estimated_receiver);
    GnssEstimatedState *estimated_state = gnss_state_estimator_get_estimated_state(receiver->state_estimator);
    gnss_estimated_state_print(estimated_state);
    printf("Done\n");
}
