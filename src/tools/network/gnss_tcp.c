#include "tools/network/gnss_tcp.h"

#include <stdlib.h>
#include "tinycsocket.h"

struct GnssTcp
{
    TcsSocket socket;
};

GnssTcp *gnss_tcp_create(void)
{
    GnssTcp *tcp = malloc(sizeof(*tcp));
    if (!tcp)
        return NULL;

    if (tcs_lib_init() != TCS_SUCCESS)
    {
        free(tcp);
        return NULL;
    }

    tcp->socket = TCS_SOCKET_INVALID;
    return tcp;
}

int gnss_tcp_connect(GnssTcp *tcp,
                     const char *host,
                     uint16_t port)
{
    if (!tcp || !host)
        return -1;

    if (tcp->socket != TCS_SOCKET_INVALID)
        return -1;

    TcsResult status = tcs_socket(
        &tcp->socket,
        TCS_FAMILY_IPV4,
        TCS_SOCKET_STREAM,
        TCS_PROTOCOL_IP_TCP);

    if (status != TCS_SUCCESS)
        return -1;

    status = tcs_connect_str(tcp->socket, host, port);

    if (status != TCS_SUCCESS)
    {
        tcs_close(&tcp->socket);
        return -1;
    }

    return 0;
}

int gnss_tcp_read(GnssTcp *tcp,
                  uint8_t *buffer,
                  size_t capacity,
                  size_t *received)
{
    if (!tcp || !buffer || !received || capacity == 0)
        return -1;

    if (tcp->socket == TCS_SOCKET_INVALID)
        return -1;

    *received = 0;

    return tcs_receive(
               tcp->socket,
               buffer,
               capacity,
               TCS_FLAG_NONE,
               received) == TCS_SUCCESS
               ? 0
               : -1;
}

int gnss_tcp_write(GnssTcp *tcp,
                   const uint8_t *buffer,
                   size_t size)
{
    if (!tcp || !buffer || size == 0)
        return -1;

    if (tcp->socket == TCS_SOCKET_INVALID)
        return -1;

    return tcs_send(
               tcp->socket,
               buffer,
               size,
               TCS_MSG_SENDALL,
               NULL) == TCS_SUCCESS
               ? 0
               : -1;
}

void gnss_tcp_destroy(GnssTcp *tcp)
{
    if (!tcp)
        return;

    if (tcp->socket != TCS_SOCKET_INVALID)
        tcs_close(&tcp->socket);

    tcs_lib_cleanup();
    free(tcp);
}
