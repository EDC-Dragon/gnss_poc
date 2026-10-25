#include "gnss_simulation.h"

#include <math.h>
#include <stddef.h>

#include "gnss_types.h"
#include "gnss_solver.h"
#include "gnss_math.h"
#include "gnss_satellite.h"
#include "gnss_receiver.h"

GnssMeasurement simulate_pseudorange_measurement(
    const Satellite *sat,
    const Receiver *true_receiver,
    double arrival_time_s)
{
    if (!sat || !true_receiver)
        return (GnssMeasurement){.status = GNSS_INVALID_ARGUMENT};

    double distance = vec3_distance(&sat->position, &true_receiver->position);

    double pseudorange = distance + true_receiver->clock_bias_m;

    if (!isfinite(pseudorange))
        return (GnssMeasurement){.status = GNSS_NUMERICAL_ERROR};

    double noise = sat->error_model.bias_m + gaussian_noise(sat->error_model.noise_sigma_m);
    double pseudorange_with_noise = pseudorange + noise;
    double sigma_m = sat->error_model.noise_sigma_m;

    double transmission_time_s = sat->time_s - distance / C_MS;

    return (GnssMeasurement){.status = GNSS_OK, .pseudorange = pseudorange_with_noise, .sigma_m = sigma_m, .transmission_time_s = transmission_time_s, .arrival_time_s = arrival_time_s};
}

void gnss_simulated_receiver_init(GnssSimulatedReceiver *receiver, Receiver true_receiver)
{
    receiver->true_receiver = true_receiver;
}

void gnss_simulated_evaluate_measurement(GnssSimulatedReceiver *simulated_receiver, double arrival_time_s, GnssSolver *solver)
{
    for (size_t i = 0; i < solver->satellite_count; i++)
    {
        Satellite *sat = solver->satellitesData[i].satellite;
        sat->time_s = arrival_time_s;

        GnssMeasurement measured_pseudorange = simulate_pseudorange_measurement(sat, &simulated_receiver->true_receiver, arrival_time_s);

        solver->satellitesData[i].measured_pseudorange = measured_pseudorange;
    }
}
