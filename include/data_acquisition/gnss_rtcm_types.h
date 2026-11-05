#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "gnss_types.h"

typedef enum
{
    RTCM_TYPE_UNKNOWN = 0,
    RTCM_TYPE_STATION = 1005,
    RTCM_TYPE_MSM7 = 1077,
    RTCM_TYPE_EPHEMERIS = 1019
} RTCMMessageType;

typedef struct
{
    int64_t station_id;
    Vec3 station_ecef;

} RTCMStationMessage;

typedef struct
{
    uint8_t satellite_id;
    uint8_t signal_id;

    double pseudorange_m;
    double cnr_dbhz;

    int32_t fine_phase_range;
    int16_t fine_phase_range_rate;
    uint16_t lock_time;
    bool half_cycle_indicator;
} RTCMMSM7Observation;

#define RTCM_MSM7_MAX_CELLS 2048

typedef struct
{
    uint16_t station_id;
    uint32_t epoch_time_ms;

    size_t observation_count;
    RTCMMSM7Observation observations[RTCM_MSM7_MAX_CELLS];
} RTCMMSM7Message;

typedef struct
{
    // Identification
    uint8_t satellite_id;
    uint16_t gps_week;
    uint8_t ura;
    uint8_t l2_code;

    // Horloge satellite
    double toc_s;
    double af0_s;
    double af1_s_per_s;
    double af2_s_per_s2;
    double tgd_s;

    // Paramètres orbitaux
    double toe_s;
    double sqrt_a_sqrt_m;
    double eccentricity;
    double m0_rad;
    double delta_n_rad_s;
    double idot_rad_s;

    double omega0_rad;
    double i0_rad;
    double omega_rad;
    double omega_dot_rad_s;

    // Corrections harmoniques
    double crs_m;
    double crc_m;
    double cuc_rad;
    double cus_rad;
    double cic_rad;
    double cis_rad;

    // État des éphémérides
    uint8_t iode;
    uint16_t iodc;
    uint8_t sv_health;
    bool l2_p_flag;
    bool fit_interval;

} RTCMEphemerisMessage;

typedef struct
{
    RTCMMessageType type;
    union
    {
        RTCMStationMessage station;
        RTCMMSM7Message msm7;
        RTCMEphemerisMessage ephemeris;
    } data;
} RTCMDecodedMessage;