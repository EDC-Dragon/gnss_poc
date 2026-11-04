#include "test_sandbox.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "network/gnss_tcp.h"
#include "data_acquisition/gnss_rctm.h"
#include "data_acquisition/gnss_rctm_decoder.h"

#include "gnss_types.h"

void test_tcp(RCTMFrameDecoder *decoder)
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

        rctm_frame_decoder_feed(decoder, buffer, received, verbose);
    }

    gnss_tcp_destroy(tcp);
}

void test_sandbox_function()
{
    RCTMFrameDecoder *decoder = rctm_frame_decoder_create();
    test_tcp(decoder);
    rctm_frame_decoder_free(decoder);
}
