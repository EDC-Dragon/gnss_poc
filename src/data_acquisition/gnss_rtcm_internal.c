#include "data_acquisition/gnss_rtcm_internal.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "gnss_types.h"
#include "data_acquisition/gnss_rtcm_tools.h"

RTCMDecodedMessage handle_station_message(const uint8_t *payload, size_t payload_size)
{
    if (payload_size < 12)
        return (RTCMDecodedMessage){.type = RTCM_TYPE_UNKNOWN};
    for (size_t i = 0; i < payload_size; i++)
        printf("%02X ", payload[i]);

    int64_t i = rtcm_get_signed_bits(payload, 12, 12);
    int64_t x = rtcm_get_signed_bits(payload, 34, 38);
    int64_t y = rtcm_get_signed_bits(payload, 74, 38);
    int64_t z = rtcm_get_signed_bits(payload, 114, 38);
    int64_t h = rtcm_get_signed_bits(payload, 152, 16);

    Vec3 station_ecef = {x * 0.0001, y * 0.0001, z * 0.0001};

    printf("i: %ld, x: %f, y: %f, z: %f, h: %ld\n", i, station_ecef.x, station_ecef.y, station_ecef.z, h);
    return (RTCMDecodedMessage){
        .type = RTCM_TYPE_STATION,
        .data.station = {
            .station_id = i,
            .station_ecef = station_ecef}};
}

typedef struct
{
    uint8_t satellite_id;           // PRN GPS (1 à 64)
    uint8_t rough_range_ms;         // 8 bits, millisecondes entières
    uint8_t extended_info;          // 4 bits
    uint16_t rough_range_modulo;    // 10 bits, fraction de milliseconde
    int16_t rough_phase_range_rate; // 14 bits, m/s
} RtcmMsm7SatelliteData;

typedef struct
{
    int32_t fine_pseudorange;
    int32_t fine_phase_range;
    uint16_t lock_time;
    bool half_cycle_indicator;
    uint16_t cnr;
    int16_t fine_phase_range_rate;
} RtcmMsm7CellData;

double msm7_pseudorange_m(
    const RtcmMsm7SatelliteData *sat,
    const RtcmMsm7CellData *cell)
{
    // Valeurs RTCM réservées : mesure indisponible
    if (sat->rough_range_ms == 255 ||
        sat->rough_range_modulo == 1023 ||
        cell->fine_pseudorange == -524288)
        return NAN;

    return 299792.458 *
           ((double)sat->rough_range_ms +
            (double)sat->rough_range_modulo / 1024.0 +
            (double)cell->fine_pseudorange / 536870912.0);
}

RTCMDecodedMessage handle_msm7_message(const uint8_t *payload, size_t payload_size) // Hard one =O
{
    // add a description of the frame
    uint64_t satellite_mask = rtcm_get_bits(payload, 73, 64);
    uint32_t signal_mask = rtcm_get_bits(payload, 137, 32);

    size_t nsat = 0;
    size_t nsig = 0;

    for (size_t i = 0; i < 64; i++)
        nsat += (satellite_mask >> i) & 1U;

    for (size_t i = 0; i < 32; i++)
        nsig += (signal_mask >> i) & 1U;

    size_t ncell = 0;

    for (size_t i = 0; i < nsat * nsig; i++)
        ncell += rtcm_get_bits(payload, 169 + i, 1);

    size_t satellite_data_offset = 169 + nsat * nsig;

    printf("Number of satellites: %zu, Number of signals: %zu, Number of cells: %zu\n", nsat, nsig, ncell);

    size_t bit_count = 0;

    RtcmMsm7SatelliteData satellites[nsat];
    size_t offset = satellite_data_offset;

    {
        size_t local_nsat = 0;
        for (size_t i = 0; i < 64; i++)
            if ((satellite_mask >> i) & 1U)
                satellites[local_nsat++].satellite_id = 63 - i; // reverse order apparently
    }

    bit_count = 8;
    for (size_t i = 0; i < nsat; i++)
        satellites[i].rough_range_ms =
            rtcm_get_bits(payload, offset + i * bit_count, bit_count);
    offset += nsat * bit_count;

    bit_count = 4;
    for (size_t i = 0; i < nsat; i++)
        satellites[i].extended_info =
            rtcm_get_bits(payload, offset + i * bit_count, bit_count);
    offset += nsat * bit_count;

    bit_count = 10;
    for (size_t i = 0; i < nsat; i++)
        satellites[i].rough_range_modulo =
            rtcm_get_bits(payload, offset + i * bit_count, bit_count);
    offset += nsat * bit_count;

    bit_count = 14;
    for (size_t i = 0; i < nsat; i++)
        satellites[i].rough_phase_range_rate =
            rtcm_get_signed_bits(payload, offset + i * bit_count, bit_count);
    offset += nsat * bit_count;

    for (size_t i = 0; i < nsat; i++)
        printf("Satellite %u: rough_range_ms=%u, extended_info=%u, rough_range_modulo=%u, rough_phase_range_rate=%d\n",
               satellites[i].satellite_id, satellites[i].rough_range_ms, satellites[i].extended_info,
               satellites[i].rough_range_modulo, satellites[i].rough_phase_range_rate);

    RtcmMsm7CellData cells[ncell];

    bit_count = 20;
    for (size_t i = 0; i < ncell; i++)
        cells[i].fine_pseudorange =
            rtcm_get_signed_bits(payload, offset + i * bit_count, bit_count);
    offset += ncell * bit_count;

    bit_count = 24;
    for (size_t i = 0; i < ncell; i++)
        cells[i].fine_phase_range =
            rtcm_get_signed_bits(payload, offset + i * bit_count, bit_count);
    offset += ncell * bit_count;

    bit_count = 10;
    for (size_t i = 0; i < ncell; i++)
        cells[i].lock_time =
            rtcm_get_bits(payload, offset + i * bit_count, bit_count); // Example bit length
    offset += ncell * bit_count;

    bit_count = 1;
    for (size_t i = 0; i < ncell; i++)
        cells[i].half_cycle_indicator =
            rtcm_get_bits(payload, offset + i * bit_count, bit_count);
    offset += ncell * bit_count;

    bit_count = 10;
    for (size_t i = 0; i < ncell; i++)
        cells[i].cnr =
            rtcm_get_bits(payload, offset + i * bit_count, bit_count);
    offset += ncell * bit_count;

    bit_count = 15;
    for (size_t i = 0; i < ncell; i++)
        cells[i].fine_phase_range_rate =
            rtcm_get_signed_bits(payload, offset + i * bit_count, bit_count);

    offset += ncell * bit_count;

    printf("Parsed %zu cells\n", ncell);
    for (size_t i = 0; i < ncell; i++)
        printf("Cell %zu: fine_pseudorange=%u, fine_phase_range=%d, lock_time=%u, half_cycle_indicator=%u, cnr=%u, fine_phase_range_rate=%d\n",
               i, cells[i].fine_pseudorange, cells[i].fine_phase_range,
               cells[i].lock_time, cells[i].half_cycle_indicator,
               cells[i].cnr, cells[i].fine_phase_range_rate);

    size_t cell_index = 0;

    for (size_t s = 0; s < nsat; s++)
    {
        size_t signal_index = 0;

        for (size_t sig = 0; sig < 32; sig++)
        {
            if (!(signal_mask & (UINT32_C(1) << (31 - sig))))
                continue;

            size_t mask_index = s * nsig + signal_index++;

            if (!rtcm_get_bits(payload, 169 + mask_index, 1))
                continue;

            RtcmMsm7SatelliteData *sat = &satellites[s];
            RtcmMsm7CellData *cell = &cells[cell_index++];

            double pseudorange_m = msm7_pseudorange_m(sat, cell);

            if (isnan(pseudorange_m))
                continue;

            printf("GPS %u, signal %zu: pseudorange = %.3f m\n",
                   sat->satellite_id, sig + 1, pseudorange_m);
        }
    }
    return (RTCMDecodedMessage){.type = RTCM_TYPE_MSM7};
}

RTCMDecodedMessage handle_rtcm_frame(const uint8_t *rtcm_payload, int payload_size)
{
    if (payload_size < 2)
        return (RTCMDecodedMessage){.type = RTCM_TYPE_UNKNOWN};

    uint16_t message_type = rtcm_get_bits(rtcm_payload, 0, 12);
    printf("Handling RTCM payload of size %d and message type %d\n", payload_size, message_type);
    switch (message_type)
    {
    case 1006:
        printf("Handling message type Station ECEF\n");
        return handle_station_message(rtcm_payload, payload_size);
        break;
    case 1077:
        printf("Handling message type MSM7\n");
        return handle_msm7_message(rtcm_payload, payload_size);
        break;
    }
    return (RTCMDecodedMessage){.type = RTCM_TYPE_UNKNOWN};
}
