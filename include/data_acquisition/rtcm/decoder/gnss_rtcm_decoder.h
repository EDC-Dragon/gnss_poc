#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "data_acquisition/rtcm/decoder/gnss_rtcm_decoder_internal.h"

#define RTCM_MAX_MESSAGES 128

typedef struct
{
    uint8_t current_frame[4096];
    int current_frame_size;
    int expected_frame_size;

    RTCMDecodedMessage messages[RTCM_MAX_MESSAGES];
    size_t message_count;

} RTCMFrameDecoder;

RTCMFrameDecoder *rtcm_frame_decoder_create(void);
void rtcm_frame_decoder_free(RTCMFrameDecoder *decoder);

void rtcm_frame_decoder_feed(RTCMFrameDecoder *decoder, const uint8_t *data, size_t data_size, bool verbose);

RTCMDecodedMessage *rtcm_frame_decoder_dequeue(RTCMFrameDecoder *decoder);
