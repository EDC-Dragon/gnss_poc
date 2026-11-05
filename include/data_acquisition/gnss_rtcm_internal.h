#pragma once

#include <stddef.h>

#include "gnss_rtcm_types.h"

RTCMDecodedMessage handle_rtcm_frame(const uint8_t *rtcm_payload, int payload_size);
