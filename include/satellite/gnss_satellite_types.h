#pragma once

#include <stdint.h>

#include "gnss_types.h"

typedef struct
{
    // Temps de référence
    uint32_t gps_week;
    double toe_s;

    // Paramètres orbitaux
    double sqrt_a_sqrt_m;
    double eccentricity;
    double m0_rad;
    double delta_n_rad_s;

    double omega0_rad;
    double i0_rad;
    double omega_rad;
    double omega_dot_rad_s;
    double idot_rad_s;

    // Corrections harmoniques
    double crs_m;
    double crc_m;
    double cuc_rad;
    double cus_rad;
    double cic_rad;
    double cis_rad;

    // Horloge satellite
    double toc_s;
    double af0_s;
    double af1_s_per_s;
    double af2_s_per_s2;
    double tgd_s;

    // Validité des éphémérides
    uint8_t iode;
    uint16_t iodc;
    uint8_t sv_health;

} GnssGpsEphemeris;

Vec3 gnss_gps_satellite_position(const GnssGpsEphemeris *eph, double gps_time_s);