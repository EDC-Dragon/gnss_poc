#include "data_acquisition/gnss_rctm_decoder.h"

#include <stdlib.h>
#include <stdio.h>

#include "data_acquisition/gnss_rctm.h"

RCTMFrameDecoder *rctm_frame_decoder_create(void)
{
    RCTMFrameDecoder *decoder = (RCTMFrameDecoder *)malloc(sizeof(RCTMFrameDecoder));
    if (decoder)
    {
        decoder->current_frame_size = 0;
        decoder->expected_frame_size = 0;
    }
    return decoder;
}

void rctm_frame_decoder_free(RCTMFrameDecoder *decoder)
{
    if (decoder)
    {
        free(decoder);
    }
}

void rctm_frame_decoder_feed(RCTMFrameDecoder *decoder, const uint8_t *data, size_t data_size, bool verbose)
{

    for (size_t current_index = 0; current_index < data_size; current_index++)
    {
        // TODO protect against header, works ATM
        if (decoder->current_frame_size == 0)
        {
            if (data[current_index] == 0xD3)
            {
                decoder->current_frame[decoder->current_frame_size] = data[current_index];
                decoder->current_frame_size = 1; // Eh
                // printf("First byte 0xD3 found at position %zu\n", current_index);
            }
        }
        else
        {
            decoder->current_frame[decoder->current_frame_size] = data[current_index];
            decoder->current_frame_size++;
            if (4096 <= decoder->current_frame_size) // Frame is too long
            {
                if (verbose)
                    printf("Frame exceeded buffer size, resetting\n");
                decoder->current_frame_size = 0;
                decoder->expected_frame_size = 0;
            }
            if (decoder->expected_frame_size == 0) // Frame payload length not yet determined
            {
                if (decoder->current_frame_size == 3) // Frame payload length can be determined now
                {
                    int current_payload_length = ((decoder->current_frame[1] & 0x03) << 8) | decoder->current_frame[2];
                    decoder->expected_frame_size = 3 + current_payload_length + 3;
                    if (verbose)
                        printf(" Expected frame size set to %d\n", decoder->expected_frame_size);
                }
                else if (3 < decoder->current_frame_size) // Error: payload length should have been determined by now (++)
                {
                    if (verbose)
                        printf("Error in payload length\n\n");
                    decoder->current_frame_size = 0;
                    decoder->expected_frame_size = 0;
                }
            }
            else // Frame payload length has been determined
            {
                if (decoder->expected_frame_size == decoder->current_frame_size) // Frame complete
                {
                    // printf("Complete frame received, size: %d\n\n", current_frame_size);
                    handle_rctm_frame(decoder->current_frame + 3, decoder->current_frame_size - 6);
                    decoder->current_frame_size = 0;
                    decoder->expected_frame_size = 0;
                    // TODO Add CRC check for the frame
                }
                else if (decoder->expected_frame_size < decoder->current_frame_size)
                {
                    if (verbose)
                        printf("Error: frame size exceeded expected size\n"); // Frame is too long
                    decoder->current_frame_size = 0;
                    decoder->expected_frame_size = 0;
                }
            }
        }
    }
}