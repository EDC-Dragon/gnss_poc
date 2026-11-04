#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint8_t current_frame[4096];
    int current_frame_size;
    int expected_frame_size;
} RCTMFrameDecoder;

RCTMFrameDecoder *rctm_frame_decoder_create(void);
void rctm_frame_decoder_free(RCTMFrameDecoder *decoder);

void rctm_frame_decoder_feed(RCTMFrameDecoder *decoder, const uint8_t *data, size_t data_size, bool verbose);