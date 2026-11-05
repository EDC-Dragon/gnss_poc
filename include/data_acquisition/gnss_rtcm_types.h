#pragma once

#include <stdint.h>

#include "gnss_types.h"

typedef enum
{
    RTCM_TYPE_UNKNOWN = 0,
    RTCM_TYPE_STATION = 1005,
    RTCM_TYPE_MSM7 = 1077
} RTCMMessageType;

typedef struct
{
    int64_t station_id;
    Vec3 station_ecef;

} RTCMStationMessage;

typedef struct
{

} RTCMMSM7Message;

typedef struct
{
    RTCMMessageType type;
    union
    {
        RTCMStationMessage station;
        RTCMMSM7Message msm7;
    } data;
} RTCMDecodedMessage;