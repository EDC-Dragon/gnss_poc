#include "test_sandbox.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "network/gnss_tcp.h"
#include "data_acquisition/gnss_rctm.h"

#include "gnss_types.h"

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
