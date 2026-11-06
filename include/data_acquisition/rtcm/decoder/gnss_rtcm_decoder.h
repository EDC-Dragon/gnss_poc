#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "data_acquisition/rtcm/decoder/gnss_rtcm_decoder_internal.h"

#define RTCM_MAX_MESSAGES 128

// RTCM frame decoder structure and functions for decoding and storing RTCM messages
typedef struct RTCMFrameDecoder RTCMFrameDecoder;

// Allocation
RTCMFrameDecoder *rtcm_frame_decoder_create(void);
// Deallocation
void rtcm_frame_decoder_free(RTCMFrameDecoder *decoder);

// Feed data into the RTCM frame decoder for processing
void rtcm_frame_decoder_feed(RTCMFrameDecoder *decoder, const uint8_t *data, size_t data_size, bool verbose);

// Dequeue the next decoded RTCM message from the decoder
// Returns:
//      NULL if no decoded message is available
RTCMDecodedMessage *rtcm_frame_decoder_dequeue(RTCMFrameDecoder *decoder);
