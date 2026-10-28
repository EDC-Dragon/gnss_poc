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
    const GnssReceiverState *true_receiver,
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

    return (GnssMeasurement){.status = GNSS_OK, .pseudorange = pseudorange_with_noise, .sigma_m = sigma_m, .satellite_id = sat->id, .satellite_position = sat->position, .transmission_time_s = transmission_time_s, .reception_time_s = reception_time_s, .availability_time_s = availability_time_s};
}

void gnss_simulated_receiver_init(GnssSimulatedReceiver *receiver, Vec3 initial_position, double initial_clock_bias_m, Vec3 initial_velocity)
{
    receiver->true_receiver.position = initial_position;
    receiver->true_receiver.time_s = 0.0;
    receiver->true_receiver.clock_bias_m = initial_clock_bias_m;
    receiver->velocity = initial_velocity;
}

void gnss_simulated_evaluate_measurement(GnssSimulatedReceiver *simulated_receiver, Satellite *satellites, size_t satellite_count, double reception_time_s, GnssReceiver *receiver)
{
    double dt = reception_time_s - simulated_receiver->true_receiver.time_s;
    if (dt < 0.0)
        return;
    simulated_receiver->true_receiver.position.x += simulated_receiver->velocity.x * dt;
    simulated_receiver->true_receiver.position.y += simulated_receiver->velocity.y * dt;
    simulated_receiver->true_receiver.position.z += simulated_receiver->velocity.z * dt;
    simulated_receiver->true_receiver.time_s = reception_time_s;
    simulated_receiver->true_receiver.clock_bias_m += 0.0; // Update clock bias if needed

    for (size_t i = 0; i < satellite_count; i++)
    {
        Satellite *sat = &satellites[i];
        sat->time_s = reception_time_s;

        GnssMeasurement measurement = simulate_pseudorange_measurement(sat, &simulated_receiver->true_receiver, reception_time_s);

        gnss_receiver_update_satellite(receiver, measurement);
    }
}
