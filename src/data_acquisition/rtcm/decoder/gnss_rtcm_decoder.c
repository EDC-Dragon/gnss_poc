#include "data_acquisition/rtcm/decoder/gnss_rtcm_decoder.h"

#include <stdlib.h>
#include <stdio.h>

#include "data_acquisition/rtcm/decoder/gnss_rtcm_decoder_internal.h"

struct RTCMFrameDecoder
{
    uint8_t current_frame[4096]; // Buffer to store the current RTCM frame being decoded
    int current_frame_size;      // Size of the current RTCM frame being decoded
    int expected_frame_size;     // Expected size of the current RTCM frame being decoded

    RTCMDecodedMessage messages[RTCM_MAX_MESSAGES]; // Array to store decoded RTCM messages
    size_t message_count;                           // Number of decoded RTCM messages stored
    RTCMDecodedMessage dequeued_message;            // Stores the most recently dequeued RTCM message, very imperfect but useful ATM
};

RTCMFrameDecoder *rtcm_frame_decoder_create(void)
{
    RTCMFrameDecoder *decoder = (RTCMFrameDecoder *)malloc(sizeof(RTCMFrameDecoder));
    if (decoder)
    {
        decoder->current_frame_size = 0;
        decoder->expected_frame_size = 0;
        decoder->message_count = 0;
    }
    return decoder;
}

void rtcm_frame_decoder_free(RTCMFrameDecoder *decoder)
{
    if (decoder)
    {
        free(decoder);
    }
}

void rtcm_frame_decoder_feed(RTCMFrameDecoder *decoder, const uint8_t *data, size_t data_size, bool verbose)
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
                    if (decoder->message_count < RTCM_MAX_MESSAGES)
                    {
                        RTCMDecodedMessage msg = handle_rtcm_frame(decoder->current_frame + 3, decoder->current_frame_size - 6, verbose);
                        if (msg.type != RTCM_TYPE_UNKNOWN)
                            decoder->messages[decoder->message_count++] = msg;
                    }
                    else
                        printf("Message buffer full, cannot store more messages\n");
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

RTCMDecodedMessage *rtcm_frame_decoder_dequeue(RTCMFrameDecoder *decoder)
{
    // TODO: Implement a real queue
    if (decoder->message_count == 0)
        return NULL;
    decoder->dequeued_message = decoder->messages[0]; // Store the most recently dequeued message
    // Shift all remaining messages one position to the left
    for (size_t i = 1; i < decoder->message_count; i++)
        decoder->messages[i - 1] = decoder->messages[i];
    decoder->message_count--;
    return &decoder->dequeued_message; // Return a pointer to the stored message
}