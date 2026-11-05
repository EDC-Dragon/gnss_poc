#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "gnss_types.h"
#include "satellite/gnss_satellite_types.h"

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
    GnssGpsEphemeris ephemeris;
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