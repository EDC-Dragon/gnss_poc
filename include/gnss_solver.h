#pragma once

#include "physical_types.h"

typedef struct
{
    Vec3 jacobian;
    double residue;
} JacobianRow;