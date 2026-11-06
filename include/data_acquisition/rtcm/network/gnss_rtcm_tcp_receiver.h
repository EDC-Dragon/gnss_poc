#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

// TCP Implementation for GNSS station RTCM data reception
typedef struct GnssRtcmTcpReceiver GnssRtcmTcpReceiver;

// Allocation
GnssRtcmTcpReceiver *gnss_rtcm_tcp_receiver_create(void);
// Deallocation
void gnss_rtcm_tcp_receiver_free(GnssRtcmTcpReceiver *receiver);

// Connect to a specified host, port, and station
// Returns:
//      true if connection was successful
//      false otherwise
bool gnss_rtcm_tcp_receiver_connect(GnssRtcmTcpReceiver *receiver, const char *host, uint16_t port, const char *station);

// Read data from connected TCP receiver into buffer
// Returns:
//      true if data was successfully read
//      false otherwise
bool gnss_rtcm_tcp_receiver_read(GnssRtcmTcpReceiver *receiver, uint8_t *buffer, size_t buffer_size, size_t *received);

// Disconnect the GNSS RTCM TCP receiver from the current connection
void gnss_rtcm_tcp_receiver_disconnect(GnssRtcmTcpReceiver *receiver);