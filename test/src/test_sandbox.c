#include "test_sandbox.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "tools/network/gnss_tcp.h"
#include "data_acquisition/rtcm/decoder/gnss_rtcm_decoder.h"
#include "gnss_core/receiver/gnss_receiver.h"
#include "data_acquisition/rtcm/network/gnss_rtcm_tcp_receiver.h"
#include "gnss_core/satellite/gnns_constellation.h"

#include "gnss_core/gnss_types.h"

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

    size_t max_measurements = 32;

    GnssMeasurement measurements[max_measurements];
    size_t measurement_count = 0;

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
                printf("Received RTCM message type STATION\n");
                printf("Station ID: %ld\n", message->data.station.station_id);
                printf("Station ECEF: x=%f, y=%f, z=%f\n", message->data.station.station_ecef.x, message->data.station.station_ecef.y, message->data.station.station_ecef.z);
                break;
            case RTCM_TYPE_MSM7:
                // printf("Received RTCM message type MSM\n");
                double epoch_time_s = message->data.msm7.epoch_time_ms / 1000.0;
                measurement_count = 0;

                for (size_t obs = 0; obs < message->data.msm7.observation_count; obs++)
                {
                    RTCMMSM7Observation *observation = &message->data.msm7.observations[obs];
                    Satellite *satellite = gnss_constellation_get_satellite(constellation, observation->satellite_id);
                    if (satellite->has_ephemeris)
                    {
                        printf("Observation %zu: satellite_id=%u, signal_id=%u, pseudorange_m=%.3f, cnr_dbhz=%.3f\n",
                               obs, observation->satellite_id, observation->signal_id,
                               observation->pseudorange_m, observation->cnr_dbhz);

                        double transmission_time_s = epoch_time_s - observation->pseudorange_m / 299792458.0;
                        Vec3 satellite_position = gnss_gps_satellite_position(&satellite->ephemeris, transmission_time_s);
                        printf("Satellite position: x=%.3f, y=%.3f, z=%.3f\n", satellite_position.x, satellite_position.y, satellite_position.z);

                        GnssMeasurement measurement;
                        measurement.status = GNSS_OK;
                        measurement.satellite_id = observation->satellite_id;
                        measurement.pseudorange_m = observation->pseudorange_m;
                        measurement.sigma_m = 5.0;
                        measurement.satellite_position = satellite_position;
                        measurement.transmission_time_s = transmission_time_s;
                        measurement.reception_time_s = epoch_time_s;
                        measurement.availability_time_s = epoch_time_s;

                        if (measurement_count < max_measurements)
                            measurements[measurement_count++] = measurement;
                    }
                }

                if (3 < measurement_count) // arbitrary
                {
                    gnss_receiver_update(receiver, measurements, measurement_count, epoch_time_s);
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
