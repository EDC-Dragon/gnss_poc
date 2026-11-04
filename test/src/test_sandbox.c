#include "test_sandbox.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "network/gnss_tcp.h"
#include "gnss_types.h"

uint64_t rtcm_get_bits(
    const uint8_t *data,
    size_t bit_offset,
    size_t bit_count) // AI, to test
{
    uint64_t result = 0;
    for (size_t i = 0; i < bit_count; i++)
    {
        size_t byte_offset = (bit_offset + i) / 8;
        size_t bit_in_byte = 7 - ((bit_offset + i) % 8);
        result <<= 1;
        result |= (data[byte_offset] >> bit_in_byte) & 0x01;
    }
    return result;
}

int64_t rtcm_get_signed_bits(
    const uint8_t *data,
    size_t bit_offset,
    size_t bit_count) // AI, to test
{
    if (bit_count == 0 || bit_count >= 64)
        return 0;

    uint64_t raw = rtcm_get_bits(data, bit_offset, bit_count);

    if (raw & (UINT64_C(1) << (bit_count - 1)))
        return (int64_t)raw - (int64_t)(UINT64_C(1) << bit_count);

    return (int64_t)raw;
}

void handle_station_message(const uint8_t *payload, size_t payload_size)
{
    if (payload_size < 12)
        return;
    for (size_t i = 0; i < payload_size; i++)
        printf("%02X ", payload[i]);

    int64_t i = rtcm_get_signed_bits(payload, 12, 12);
    int64_t x = rtcm_get_signed_bits(payload, 34, 38);
    int64_t y = rtcm_get_signed_bits(payload, 74, 38);
    int64_t z = rtcm_get_signed_bits(payload, 114, 38);
    int64_t h = rtcm_get_signed_bits(payload, 152, 16);

    Vec3 station_ecef = {x * 0.0001, y * 0.0001, z * 0.0001};

    printf("i: %ld, x: %f, y: %f, z: %f, h: %ld\n", i, station_ecef.x, station_ecef.y, station_ecef.z, h);
}

void handle_rctm_frame(const uint8_t *rctm_payload, int payload_size)
{
    if (payload_size < 2)
        return;

    uint16_t message_type = rtcm_get_bits(rctm_payload, 0, 12);
    printf("Handling RTCM payload of size %d and message type %d\n", payload_size, message_type);
    switch (message_type)
    {
    case 1006:
        printf("Handling message type Station ECEF\n");
        handle_station_message(rctm_payload, payload_size);
        break;
    }
}

void test_tcp(void)
{
    GnssTcp *tcp = gnss_tcp_create();
    if (!tcp)
        return;

    if (gnss_tcp_connect(tcp, "crtk.net", 2101) != 0)
    {
        gnss_tcp_destroy(tcp);
        return;
    }

    const char *request =
        "GET /SAOG HTTP/1.0\r\n"
        "Host: crtk.net\r\n"
        "User-Agent: NTRIP gnss_poc/0.1\r\n"
        "\r\n";

    if (gnss_tcp_write(tcp,
                       (const uint8_t *)request,
                       strlen(request)) != 0)
    {
        gnss_tcp_destroy(tcp);
        return;
    }

    uint8_t current_frame[4096];
    int current_frame_size = 0;
    int expected_frame_size = 0;

    uint8_t buffer[4096];
    size_t received;

    bool verbose = false;

    while (gnss_tcp_read(tcp, buffer, sizeof(buffer), &received) == 0)
    {
        if (received == 0)
            break;

        if (verbose)
        {
            printf("Received %zu bytes\n", received);
            for (size_t i = 0; i < received && i < 16; i++)
                printf("%02X ", buffer[i]);
            printf("\n");
        }

        for (size_t current_index = 0; current_index < received; current_index++)
        {
            // TODO protect against header, works ATM
            if (current_frame_size == 0)
            {
                if (buffer[current_index] == 0xD3)
                {
                    current_frame[current_frame_size] = buffer[current_index];
                    current_frame_size = 1; // Eh
                    // printf("First byte 0xD3 found at position %zu\n", current_index);
                }
            }
            else
            {
                current_frame[current_frame_size] = buffer[current_index];
                current_frame_size++;
                if (4096 <= current_frame_size) // Frame is too long
                {
                    if (verbose)
                        printf("Frame exceeded buffer size, resetting\n");
                    current_frame_size = 0;
                    expected_frame_size = 0;
                }
                if (expected_frame_size == 0) // Frame payload length not yet determined
                {
                    if (current_frame_size == 3) // Frame payload length can be determined now
                    {
                        int current_payload_length = ((current_frame[1] & 0x03) << 8) | current_frame[2];
                        expected_frame_size = 3 + current_payload_length + 3;
                        if (verbose)
                            printf(" Expected frame size set to %d\n", expected_frame_size);
                    }
                    else if (3 < current_frame_size) // Error: payload length should have been determined by now (++)
                    {
                        if (verbose)
                            printf("Error in payload length\n\n");
                        current_frame_size = 0;
                        expected_frame_size = 0;
                    }
                }
                else // Frame payload length has been determined
                {
                    if (expected_frame_size == current_frame_size) // Frame complete
                    {
                        // printf("Complete frame received, size: %d\n\n", current_frame_size);
                        handle_rctm_frame(current_frame + 3, current_frame_size - 6);
                        current_frame_size = 0;
                        expected_frame_size = 0;
                        // TODO Add CRC check for the frame
                    }
                    else if (expected_frame_size < current_frame_size)
                    {
                        if (verbose)
                            printf("Error: frame size exceeded expected size\n"); // Frame is too long
                        current_frame_size = 0;
                        expected_frame_size = 0;
                    }
                }
            }
        }
    }

    gnss_tcp_destroy(tcp);
}

void test_sandbox_function()
{
    test_tcp();
}
