#include "receiver/estimator/gnss_state_estimator_types.h"

#include <stdio.h>

void gnss_estimated_state_print(const GnssEstimatedState *state)
{
    printf("Position: [%f, %f, %f]\n", state->position.x, state->position.y, state->position.z);
    printf("Velocity: [%f, %f, %f]\n", state->velocity.x, state->velocity.y, state->velocity.z);
    printf("Clock Bias: %f m\n", state->clock_bias_m);
    printf("Time: %f s\n", state->time_s);
}