#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct GnssTcp GnssTcp;

GnssTcp *gnss_tcp_create(void);

int gnss_tcp_connect(GnssTcp *tcp,
                     const char *host,
                     uint16_t port);

int gnss_tcp_read(GnssTcp *tcp,
                  uint8_t *buffer,
                  size_t capacity,
                  size_t *received);

int gnss_tcp_write(GnssTcp *tcp,
                   const uint8_t *buffer,
                   size_t size);

void gnss_tcp_destroy(GnssTcp *tcp);
