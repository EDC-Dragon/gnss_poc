#include "simulation/gnss_simulation.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#include "gnss_types.h"
#include "gnss_math.h"
#include "satellite/gnss_satellite.h"
#include "receiver/gnss_receiver.h"
#include "receiver/solver/gnss_solver.h"
#include "simulation/gnss_simulation_csv_reader.h"

GnssMeasurement simulate_pseudorange_measurement(
    const Satellite *sat,
    const GnssPositionClockState *true_receiver,
    double reception_time_s)
{
    if (!sat || !true_receiver)
        return (GnssMeasurement){.status = GNSS_INVALID_ARGUMENT};

    double distance = vec3_distance(&sat->position, &true_receiver->position);

    double pseudorange = distance + true_receiver->clock_bias_m;

    if (!isfinite(pseudorange))
        return (GnssMeasurement){.status = GNSS_NUMERICAL_ERROR};

    double noise = sat->error_model.bias_m + gaussian_noise_at_time(sat->id, reception_time_s, sat->error_model.noise_sigma_m);
    double pseudorange_with_noise = pseudorange + noise;
    double sigma_m = sat->error_model.noise_sigma_m;

    double transmission_time_s = reception_time_s - distance / C_MS;

    double processing_delay_s = 0.0;
    double availability_time_s = reception_time_s + processing_delay_s;

    return (GnssMeasurement){.status = GNSS_OK, .pseudorange_m = pseudorange_with_noise, .sigma_m = sigma_m, .satellite_id = sat->id, .satellite_position = sat->position, .transmission_time_s = transmission_time_s, .reception_time_s = reception_time_s, .availability_time_s = availability_time_s};
}

GnssSimulator *gnss_simulator_create()
{
    return malloc(sizeof(GnssSimulator));
}
void gnss_simulator_free(GnssSimulator *sim)
{
    if (sim->data)
        free(sim->data);
    free(sim);
}

void gnss_simulator_initialize(GnssSimulator *sim, const char *path)
{
    char file[1024];
    sprintf(file, "%s/measurements.csv", path);
    sim->data = read_simulation_csv(file);
    sim->current_epoch = 0;
}

GnssSimulationEpoch *gnss_simulator_step(GnssSimulator *sim)
{
    if (sim->data->epoch_count <= sim->current_epoch)
        return NULL;
    return sim->data->epochs[sim->current_epoch++];
}