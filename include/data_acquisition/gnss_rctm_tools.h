#pragma once

#include <stdint.h>
#include <stddef.h>

uint64_t rtcm_get_bits(
    const uint8_t *data,
    size_t bit_offset,
    size_t bit_count);

int64_t rtcm_get_signed_bits(
    const uint8_t *data,
    size_t bit_offset,
    size_t bit_count);
