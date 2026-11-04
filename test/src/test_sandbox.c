#include "test_sandbox.h"

#include <stdio.h>
#include <string.h>
#include "network/gnss_tcp.h"

void test_tcp(void)
{
    GnssTcp *tcp = gnss_tcp_create();
    if (!tcp)
        return;

    if (gnss_tcp_connect(tcp, "example.com", 80) != 0)
    {
        gnss_tcp_destroy(tcp);
        return;
    }

    const char *request =
        "GET / HTTP/1.1\r\n"
        "Host: example.com\r\n"
        "Connection: close\r\n"
        "\r\n";

    if (gnss_tcp_write(tcp,
                       (const uint8_t *)request,
                       strlen(request)) != 0)
    {
        gnss_tcp_destroy(tcp);
        return;
    }

    uint8_t buffer[4096];
    size_t received = 0;

    if (gnss_tcp_read(tcp, buffer,
                      sizeof(buffer), &received) == 0)
    {
        printf("Received %zu bytes\n", received);
        fwrite(buffer, 1, received, stdout);
        printf("\n");
    }

    gnss_tcp_destroy(tcp);
}

void test_sandbox_function()
{
    test_tcp();
}
