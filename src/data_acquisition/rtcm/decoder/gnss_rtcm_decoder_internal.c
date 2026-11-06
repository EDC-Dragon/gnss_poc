#include "data_acquisition/rtcm/decoder/gnss_rtcm_decoder_internal.h"

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "gnss_core/gnss_types.h"
#include "tools/gnss_byte_tools.h"

RTCMDecodedMessage handle_station_message(const uint8_t *payload, size_t payload_size, bool verbose)
{
    if (payload_size < 12)
        return (RTCMDecodedMessage){.type = RTCM_TYPE_UNKNOWN};
    if (verbose)
        for (size_t i = 0; i < payload_size; i++)
            printf("%02X ", payload[i]);

    int64_t i = rtcm_get_signed_bits(payload, 12, 12);
    int64_t x = rtcm_get_signed_bits(payload, 34, 38);
    int64_t y = rtcm_get_signed_bits(payload, 74, 38);
    int64_t z = rtcm_get_signed_bits(payload, 114, 38);
    int64_t h = rtcm_get_signed_bits(payload, 152, 16);

    Vec3 station_ecef = {x * 0.0001, y * 0.0001, z * 0.0001};

    if (verbose)
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

RTCMDecodedMessage handle_msm7_message(const uint8_t *payload, size_t payload_size, bool verbose) // Hard one =O
{
    (void)payload_size;

    RTCMDecodedMessage result = {
        .type = RTCM_TYPE_MSM7};

    RTCMMSM7Message *msm7 = &result.data.msm7;

    msm7->station_id = rtcm_get_bits(payload, 12, 12);
    msm7->epoch_time_ms = rtcm_get_bits(payload, 24, 30);

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

    if (verbose)
        printf("Number of satellites: %zu, Number of signals: %zu, Number of cells: %zu\n", nsat, nsig, ncell);

    size_t bit_count = 0;

    RtcmMsm7SatelliteData satellites[nsat];
    size_t offset = satellite_data_offset;

    {
        size_t local_nsat = 0;
        for (size_t i = 0; i < 64; i++)
        {
            if ((satellite_mask >> (63 - i)) & 1U)
                satellites[local_nsat++].satellite_id = i + 1;
        }
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

    if (verbose)
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

    if (verbose)
        printf("Parsed %zu cells\n", ncell);
    if (verbose)
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

            RTCMMSM7Observation *obs =
                &msm7->observations[msm7->observation_count++];

            *obs = (RTCMMSM7Observation){
                .satellite_id = sat->satellite_id,
                .signal_id = sig + 1,
                .pseudorange_m = pseudorange_m,
                .cnr_dbhz = cell->cnr / 16.0,
                .fine_phase_range = cell->fine_phase_range,
                .fine_phase_range_rate = cell->fine_phase_range_rate,
                .lock_time = cell->lock_time,
                .half_cycle_indicator = cell->half_cycle_indicator};
        }
    }

    return result;
}

RTCMDecodedMessage handle_ephemeris_message(const uint8_t *rtcm_payload, int payload_size, bool verbose)
{
    RTCMDecodedMessage result = {
        .type = RTCM_TYPE_UNKNOWN};

    // RTCM 1019 : 488 bits = 61 octets
    if (!rtcm_payload || payload_size < 61)
        return result;

    if (rtcm_get_bits(rtcm_payload, 0, 12) != 1019)
        return result;

    result.type = RTCM_TYPE_EPHEMERIS;

    RTCMEphemerisMessage *eph = &result.data.ephemeris;
#define GPS_PI 3.14159265358979323846

    // Identification satellite
    eph->satellite_id = rtcm_get_bits(rtcm_payload, 12, 6);
    eph->ephemeris.gps_week = rtcm_get_bits(rtcm_payload, 18, 10);
    // uint8_t ura = rtcm_get_bits(rtcm_payload, 28, 4);
    // uint8_t l2_code = rtcm_get_bits(rtcm_payload, 32, 2);

    // Paramètres d'horloge et indices
    eph->ephemeris.idot_rad_s = rtcm_get_signed_bits(rtcm_payload, 34, 14) * 0x1p-43 * GPS_PI;
    eph->ephemeris.iode = rtcm_get_bits(rtcm_payload, 48, 8);

    eph->ephemeris.toc_s = rtcm_get_bits(rtcm_payload, 56, 16) * 16.0;
    eph->ephemeris.af2_s_per_s2 = rtcm_get_signed_bits(rtcm_payload, 72, 8) * 0x1p-55;
    eph->ephemeris.af1_s_per_s = rtcm_get_signed_bits(rtcm_payload, 80, 16) * 0x1p-43;
    eph->ephemeris.af0_s = rtcm_get_signed_bits(rtcm_payload, 96, 22) * 0x1p-31;

    eph->ephemeris.iodc = rtcm_get_bits(rtcm_payload, 118, 10);

    // Paramètres orbitaux
    eph->ephemeris.crs_m = rtcm_get_signed_bits(rtcm_payload, 128, 16) * 0x1p-5;
    eph->ephemeris.delta_n_rad_s = rtcm_get_signed_bits(rtcm_payload, 144, 16) * 0x1p-43 * GPS_PI;
    eph->ephemeris.m0_rad = rtcm_get_signed_bits(rtcm_payload, 160, 32) * 0x1p-31 * GPS_PI;
    eph->ephemeris.cuc_rad = rtcm_get_signed_bits(rtcm_payload, 192, 16) * 0x1p-29;

    eph->ephemeris.eccentricity = rtcm_get_bits(rtcm_payload, 208, 32) * 0x1p-33;

    eph->ephemeris.cus_rad = rtcm_get_signed_bits(rtcm_payload, 240, 16) * 0x1p-29;
    eph->ephemeris.sqrt_a_sqrt_m = rtcm_get_bits(rtcm_payload, 256, 32) * 0x1p-19;
    eph->ephemeris.toe_s = rtcm_get_bits(rtcm_payload, 288, 16) * 16.0;

    eph->ephemeris.cic_rad = rtcm_get_signed_bits(rtcm_payload, 304, 16) * 0x1p-29;
    eph->ephemeris.omega0_rad = rtcm_get_signed_bits(rtcm_payload, 320, 32) * 0x1p-31 * GPS_PI;
    eph->ephemeris.cis_rad = rtcm_get_signed_bits(rtcm_payload, 352, 16) * 0x1p-29;
    eph->ephemeris.i0_rad = rtcm_get_signed_bits(rtcm_payload, 368, 32) * 0x1p-31 * GPS_PI;
    eph->ephemeris.crc_m = rtcm_get_signed_bits(rtcm_payload, 400, 16) * 0x1p-5;
    eph->ephemeris.omega_rad = rtcm_get_signed_bits(rtcm_payload, 416, 32) * 0x1p-31 * GPS_PI;

    eph->ephemeris.omega_dot_rad_s = rtcm_get_signed_bits(rtcm_payload, 448, 24) * 0x1p-43 * GPS_PI;

    // État satellite
    eph->ephemeris.tgd_s = rtcm_get_signed_bits(rtcm_payload, 472, 8) * 0x1p-31;
    eph->ephemeris.sv_health = rtcm_get_bits(rtcm_payload, 480, 6);
    // bool l2_p_flag = rtcm_get_bits(rtcm_payload, 486, 1);
    // bool fit_interval = rtcm_get_bits(rtcm_payload, 487, 1);

    // Debug
    if (verbose)
    {
        printf("GPS PRN: %u\n", eph->satellite_id);
        printf("GPS week (10 bits): %u\n", eph->ephemeris.gps_week);
        printf("IODE: %u\n", eph->ephemeris.iode);
        printf("Toe: %.0f s\n", eph->ephemeris.toe_s);
        printf("sqrtA: %.6f sqrt(m)\n", eph->ephemeris.sqrt_a_sqrt_m);
        printf("Semi-major axis: %.3f km\n",
               eph->ephemeris.sqrt_a_sqrt_m * eph->ephemeris.sqrt_a_sqrt_m / 1000.0);
        printf("Eccentricity: %.9f\n", eph->ephemeris.eccentricity);
    }

    return result;
}

RTCMDecodedMessage handle_rtcm_frame(const uint8_t *rtcm_payload, int payload_size, bool verbose)
{
    if (payload_size < 2)
        return (RTCMDecodedMessage){.type = RTCM_TYPE_UNKNOWN};

    uint16_t message_type = rtcm_get_bits(rtcm_payload, 0, 12);
    if (verbose)
        printf("Handling RTCM payload of size %d and message type %d\n", payload_size, message_type);
    switch (message_type)
    {
    case 1006:
        if (verbose)
            printf("Handling message type Station ECEF\n");
        return handle_station_message(rtcm_payload, payload_size, verbose);
    case 1077:
        if (verbose)
            printf("Handling message type MSM7\n");
        return handle_msm7_message(rtcm_payload, payload_size, verbose);
    case 1019:
        if (verbose)
            printf("Handling message type 1019\n");
        return handle_ephemeris_message(rtcm_payload, payload_size, verbose);
    default:
        return (RTCMDecodedMessage){.type = RTCM_TYPE_UNKNOWN};
        break;
    }
}
