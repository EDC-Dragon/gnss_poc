#include "test_sandbox.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "network/gnss_tcp.h"
#include "data_acquisition/gnss_rtcm_decoder.h"
#include "receiver/gnss_receiver.h"
#include "data_acquisition/gnss_rtcm_tcp_receiver.h"
#include "satellite/gnns_constellation.h"

#include "gnss_types.h"

void test_sandbox_function()
{
    // Receiver
    GnssReceiver *receiver = gnss_receiver_create(64);
    gnss_receiver_initialize(receiver);

    // Satellites
    GnssConstellation *constellation = gnss_constellation_create();
    if (!constellation)
        return;

    // Communication
    GnssRtcmTcpReceiver *tcp_receiver = gnss_rtcm_tcp_receiver_create();
    if (!tcp_receiver)
        return;

    if (!gnss_rtcm_tcp_receiver_connect(tcp_receiver, "crtk.net", 2101, "SAOG"))
        return;

    // Decoder
    RTCMFrameDecoder *decoder = rtcm_frame_decoder_create();

    uint8_t buffer[4096];
    size_t received;

    for (int i = 0; i < 100; i++)
    {
        if (gnss_rtcm_tcp_receiver_read(tcp_receiver, buffer, sizeof(buffer), &received) == false)
            break;

        rtcm_frame_decoder_feed(decoder, buffer, received, false);

        RTCMDecodedMessage *message = rtcm_frame_decoder_dequeue(decoder);
        while (message != NULL)
        {
            switch (message->type)
            {
            case RTCM_TYPE_STATION:
                // printf("Received RTCM message type STATION\n");
                break;
            case RTCM_TYPE_MSM7:
                // printf("Received RTCM message type MSM\n");
                if (0)
                    for (size_t obs = 0; obs < message->data.msm7.observation_count; obs++)
                    {
                        RTCMMSM7Observation *observation = &message->data.msm7.observations[obs];
                        printf("Observation %zu: satellite_id=%u, signal_id=%u, pseudorange_m=%.3f, cnr_dbhz=%.3f\n",
                               obs, observation->satellite_id, observation->signal_id,
                               observation->pseudorange_m, observation->cnr_dbhz);
                    }
                break;
            case RTCM_TYPE_EPHEMERIS:
                // printf("Received RTCM message type EPHEMERIS\n");
                gnss_constellation_notify_ephemeris(constellation, message->data.ephemeris.satellite_id, message->data.ephemeris.ephemeris);
                break;
            default:
                break;
            }
            message = rtcm_frame_decoder_dequeue(decoder);
        }
    }

    // Free decoder
    rtcm_frame_decoder_free(decoder);

    // Free communication
    gnss_rtcm_tcp_receiver_disconnect(tcp_receiver);
    gnss_rtcm_tcp_receiver_free(tcp_receiver);

    // Free receiver
    gnss_receiver_free(receiver);
}
