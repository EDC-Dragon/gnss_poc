#include "receiver/gnss_receiver.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "receiver/estimator/gnss_state_estimator_types.h"

struct GnssReceiver
{
    // GnssPositionClockState estimated_receiver;

    GnssSolver *solver;

    GnssStateEstimator *state_estimator;

    GnssMeasurement *measurements;
    size_t satellite_count;
    size_t max_satellite_count;
};

GnssReceiver *gnss_receiver_create(size_t max_satellite_count)
{
    GnssReceiver *receiver = (GnssReceiver *)malloc(sizeof(GnssReceiver));
    receiver->measurements = (GnssMeasurement *)malloc(max_satellite_count * sizeof(GnssMeasurement));
    receiver->satellite_count = 0;
    receiver->max_satellite_count = max_satellite_count;
    receiver->solver = gnss_solver_create(max_satellite_count);
    receiver->state_estimator = gnss_state_estimator_create();
    return receiver;
}

void gnss_receiver_initialize(GnssReceiver *receiver)
{
    for (size_t i = 0; i < receiver->satellite_count; i++)
    {
        receiver->measurements[i] = (GnssMeasurement){0};
    }
    gnss_solver_initialize(receiver->solver);
    gnss_state_estimator_initialize(receiver->state_estimator);
}

void gnss_receiver_free(GnssReceiver *receiver)
{
    free(receiver->measurements);
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
        if (receiver->measurements[i].satellite_id == satellite_id)
            return (GnssSatelliteSearchResult){true, i};
    return (GnssSatelliteSearchResult){false, 0};
}

void gnss_receiver_link_satellite(GnssReceiver *receiver, SatelliteId satellite_id, GnssMeasurement measurement)
{
    GnssSatelliteSearchResult res = gnss_receiver_find_satellite(receiver, satellite_id);
    if (!res.found && receiver->satellite_count < receiver->max_satellite_count)
    {
        receiver->measurements[receiver->satellite_count] = measurement;
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
            receiver->measurements[i] = receiver->measurements[i + 1];
    }
}

void gnss_receiver_unlink_obsolete_satellites(GnssReceiver *receiver, double time_s, double satellite_obsolescence_age_s)
{
    // Reverse order to avoid shifting issues
    for (size_t i = receiver->satellite_count; i > 0; i--)
    {
        double age = time_s - receiver->measurements[i - 1].reception_time_s;
        if (age > satellite_obsolescence_age_s)
            gnss_receiver_unlink_satellite(receiver, receiver->measurements[i - 1].satellite_id);
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
        receiver->measurements[res.index] = measurement;
    }
}

void gnss_receiver_update(GnssReceiver *receiver, double time_s)
{
    printf("------------------------------------------\n");
    printf("Updating GNSS receiver at time %f\n", time_s);
    double satellite_obsolescence_age_s = 10.0;
    gnss_receiver_unlink_obsolete_satellites(receiver, time_s, satellite_obsolescence_age_s);

    GnssMeasurement valid_measurements[receiver->satellite_count];
    size_t valid_satellite_count = 0;

    double epoch_time_s = 0.0;
    for (size_t i = 0; i < receiver->satellite_count; i++)
    {
        GnssMeasurement *m = &receiver->measurements[i];
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
        GnssMeasurement *m = &receiver->measurements[i];
        if (m->status != GNSS_OK)
            continue;
        if (m->availability_time_s > time_s)
            continue;
        if (fabs(m->reception_time_s - epoch_time_s) > 1e-6)
            continue;
        valid_measurements[valid_satellite_count] = receiver->measurements[i];
        valid_satellite_count++;
    }

    if (valid_satellite_count < 4)
    {
        printf("Not enough valid satellites\n");
        return;
    }

    printf("Solving...\n");
    gnss_solver_solve(receiver->solver, 100, 10e-7, valid_measurements, valid_satellite_count, false);
    gnss_solver_print_estimated_receiver(receiver->solver);
    GnssPositionClockState *estimated_state_from_solver = gnss_solver_get_estimated_state(receiver->solver);

    printf("State estimation...\n");
    gnss_state_estimator_update(receiver->state_estimator, estimated_state_from_solver);

    GnssEstimatedState *estimated_state = gnss_state_estimator_get_estimated_state(receiver->state_estimator);
    gnss_estimated_state_print(estimated_state);
    printf("Done\n");
}
