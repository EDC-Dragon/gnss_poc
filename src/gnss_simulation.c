#include "gnss_simulation.h"

#include "gnss_solver.h"
#include "physical_types.h"
#include "gnss_math.h"

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
