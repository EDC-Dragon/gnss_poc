#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct GnssRtcmTcpReceiver GnssRtcmTcpReceiver;

GnssRtcmTcpReceiver *gnss_rtcm_tcp_receiver_create(void);
void gnss_rtcm_tcp_receiver_free(GnssRtcmTcpReceiver *receiver);

bool gnss_rtcm_tcp_receiver_connect(GnssRtcmTcpReceiver *receiver, const char *host, uint16_t port, const char *station);

bool gnss_rtcm_tcp_receiver_read(GnssRtcmTcpReceiver *receiver, uint8_t *buffer, size_t buffer_size, size_t *received);

void gnss_rtcm_tcp_receiver_disconnect(GnssRtcmTcpReceiver *receiver);