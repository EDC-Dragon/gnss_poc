#include "tools/gnss_byte_tools.h"

#include <stdint.h>

uint64_t rtcm_get_bits(
    const uint8_t *data,
    size_t bit_offset,
    size_t bit_count) // AI, to test
{
    uint64_t result = 0;
    for (size_t i = 0; i < bit_count; i++)
    {
        size_t byte_offset = (bit_offset + i) / 8;
        size_t bit_in_byte = 7 - ((bit_offset + i) % 8);
        result <<= 1;
        result |= (data[byte_offset] >> bit_in_byte) & 0x01;
    }
    return result;
}

int64_t rtcm_get_signed_bits(
    const uint8_t *data,
    size_t bit_offset,
    size_t bit_count) // AI, to test
{
    if (bit_count == 0 || bit_count >= 64)
        return 0;

    uint64_t raw = rtcm_get_bits(data, bit_offset, bit_count);

    if (raw & (UINT64_C(1) << (bit_count - 1)))
        return (int64_t)raw - (int64_t)(UINT64_C(1) << bit_count);

    return (int64_t)raw;
}
