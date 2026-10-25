#pragma once

#include "gnss_types.h"

typedef struct
{
    Vec3 position;
    double clock_bias_m;
} Receiver;
