#include "gnss_simulation.h"

#include "gnss_solver.h"
#include "physical_types.h"
#include "gnss_math.h"

void gnss_simulated_receiver_init(GnssSimulatedReceiver *receiver, Receiver true_receiver)
{
    receiver->true_receiver = true_receiver;
}

void gnss_simulated_evaluate_measurement(GnssSimulatedReceiver *simulated_receiver, GnssSolver *solver)
{
    for (size_t i = 0; i < solver->satellite_count; i++)
    {
        Satellite *sat = solver->satellitesData[i].satellite;
        GnssResult measured_pseudorange = compute_pseudorange(sat, &simulated_receiver->true_receiver);

        if (measured_pseudorange.status == GNSS_OK)
        {
            double noise = sat->error_model.bias_m + gaussian_noise(sat->error_model.noise_sigma_m);
            measured_pseudorange.pseudorange += noise;
            measured_pseudorange.sigma_m = sat->error_model.noise_sigma_m;
        }

        solver->satellitesData[i].measured_pseudorange = measured_pseudorange;
    }
}
