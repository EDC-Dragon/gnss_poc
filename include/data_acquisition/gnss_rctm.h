#pragma once

#include <stdint.h>
#include <stddef.h>

void handle_rctm_frame(const uint8_t *rctm_payload, int payload_size);
