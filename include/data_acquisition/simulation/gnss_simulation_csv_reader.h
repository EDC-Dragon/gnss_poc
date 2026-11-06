#pragma once

#include <stdio.h>
#include "data_acquisition/simulation/gnss_simulation_types.h"

GnssSimulationData *read_simulation_csv(const char *path);