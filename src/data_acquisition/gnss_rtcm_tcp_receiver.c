#include "data_acquisition/gnss_rtcm_tcp_receiver.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "network/gnss_tcp.h"

struct GnssRtcmTcpReceiver
{
    GnssTcp *tcp;
};

GnssRtcmTcpReceiver *gnss_rtcm_tcp_receiver_create(void)
{
    GnssRtcmTcpReceiver *receiver = (GnssRtcmTcpReceiver *)malloc(sizeof(GnssRtcmTcpReceiver));
    if (receiver)
    {
        receiver->tcp = NULL;
    }
    return receiver;
}

void gnss_rtcm_tcp_receiver_free(GnssRtcmTcpReceiver *receiver)
{
    if (receiver)
    {
        free(receiver);
    }
}

bool gnss_rtcm_tcp_receiver_connect(GnssRtcmTcpReceiver *receiver, const char *host, uint16_t port, const char *station)
{
    receiver->tcp = gnss_tcp_create();
    if (!receiver->tcp)
        return false;

    if (gnss_tcp_connect(receiver->tcp, host, port) != 0)
    {
        gnss_tcp_destroy(receiver->tcp);
        receiver->tcp = NULL;
        return false;
    }

    char request[1024];
    snprintf(request, 1024,
             "GET /%s HTTP/1.0\r\n"
             "Host: %s\r\n"
             "User-Agent: NTRIP gnss_poc/0.1\r\n"
             "\r\n",
             station, host);

    if (gnss_tcp_write(receiver->tcp,
                       (const uint8_t *)request,
                       strlen(request)) != 0)
    {
        gnss_tcp_destroy(receiver->tcp);
        receiver->tcp = NULL;
        return false;
    }
    return true;
}

bool gnss_rtcm_tcp_receiver_read(GnssRtcmTcpReceiver *receiver, uint8_t *buffer, size_t buffer_size, size_t *received)
{
    if (!receiver || !receiver->tcp)
        return false;

    int ret = gnss_tcp_read(receiver->tcp, buffer, buffer_size, received);
    if (ret != 0)
        return false;

    return true;
}

void gnss_rtcm_tcp_receiver_disconnect(GnssRtcmTcpReceiver *receiver)
{
    if (receiver && receiver->tcp)
    {
        gnss_tcp_destroy(receiver->tcp);
        receiver->tcp = NULL;
    }
}