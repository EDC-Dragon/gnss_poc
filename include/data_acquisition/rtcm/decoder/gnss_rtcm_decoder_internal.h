#pragma once

#include <stddef.h>
#include <stdbool.h>

#include "data_acquisition/rtcm/gnss_rtcm_types.h"

RTCMDecodedMessage handle_rtcm_frame(const uint8_t *rtcm_payload, int payload_size, bool verbose);
