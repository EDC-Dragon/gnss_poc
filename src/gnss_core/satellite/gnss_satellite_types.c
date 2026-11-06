#include "gnss_core/satellite/gnss_satellite_types.h"

#include <math.h>

#define GPS_MU 3.986005e14
#define GPS_OMEGA_E 7.2921151467e-5
#define GPS_WEEK_S 604800.0

Vec3 gnss_gps_satellite_position(
    const GnssGpsEphemeris *eph,
    double gps_time_s)
{
    Vec3 invalid = {NAN, NAN, NAN};

    if (!eph)
        return invalid;

    double sqrt_a = eph->sqrt_a_sqrt_m;
    double a = sqrt_a * sqrt_a;
    double e = eph->eccentricity;

    if (a <= 0.0 || e < 0.0 || e >= 1.0)
        return invalid;

    // Temps depuis l'époque de référence orbitale
    double tk = gps_time_s - eph->toe_s;

    // Gestion du changement de semaine GPS
    if (tk > GPS_WEEK_S / 2.0)
        tk -= GPS_WEEK_S;
    else if (tk < -GPS_WEEK_S / 2.0)
        tk += GPS_WEEK_S;

    // Mouvement moyen et anomalie moyenne
    double n0 = sqrt(GPS_MU / (a * a * a));
    double n = n0 + eph->delta_n_rad_s;
    double M = eph->m0_rad + n * tk;

    // Résolution de l'équation de Kepler (Newton-Raphson)
    double E = M;

    for (int j = 0; j < 20; j++)
    {
        double delta = (M - E + e * sin(E)) /
                       (1.0 - e * cos(E));

        E += delta;

        if (fabs(delta) < 1e-13)
            break;
    }

    // Anomalie vraie
    double v = atan2(
        sqrt(1.0 - e * e) * sin(E),
        cos(E) - e);

    double phi = v + eph->omega_rad;
    double sin2phi = sin(2.0 * phi);
    double cos2phi = cos(2.0 * phi);

    // Corrections harmoniques
    double u = phi + eph->cus_rad * sin2phi + eph->cuc_rad * cos2phi;

    double r = a * (1.0 - e * cos(E)) + eph->crs_m * sin2phi + eph->crc_m * cos2phi;

    double i = eph->i0_rad + eph->idot_rad_s * tk + eph->cis_rad * sin2phi + eph->cic_rad * cos2phi;

    // Position dans le plan orbital
    double xp = r * cos(u);
    double yp = r * sin(u);

    // Longitude du nœud ascendant
    double omega = eph->omega0_rad + (eph->omega_dot_rad_s - GPS_OMEGA_E) * tk - GPS_OMEGA_E * eph->toe_s;

    // Transformation vers ECEF
    return (Vec3){
        .x = xp * cos(omega) - yp * cos(i) * sin(omega),
        .y = xp * sin(omega) + yp * cos(i) * cos(omega),
        .z = yp * sin(i)};
}