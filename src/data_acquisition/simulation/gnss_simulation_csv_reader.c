#include "data_acquisition/simulation/gnss_simulation_csv_reader.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "gnss_core/gnss_types.h"
#include "tools/gnss_string.h"

double processing_delay_s = 0.0;

typedef enum
{
    CSV_TRANSMISSION_TIME = 0,
    CSV_RECEPTION_TIME = 1,
    CSV_SATELLITE_ID = 2,
    CSV_SATELLITE_X = 3,
    CSV_SATELLITE_Y = 4,
    CSV_SATELLITE_Z = 5,
    CSV_PSEUDORANGE = 6,
    CSV_SIGMA = 7,
    CSV_RECEIVER_POSITION_X = 8,
    CSV_RECEIVER_POSITION_Y = 9,
    CSV_RECEIVER_POSITION_Z = 10,
    CSV_RECEIVER_VELOCITY_X = 11,
    CSV_RECEIVER_VELOCITY_Y = 12,
    CSV_RECEIVER_VELOCITY_Z = 13
} CsvField;

#define CSV_FIELD_COUNT 14

static const char *CSV_FIELD_NAMES[CSV_FIELD_COUNT] = {
    "transmission_time_s",
    "reception_time_s",
    "satellite_id",
    "satellite_position_x",
    "satellite_position_y",
    "satellite_position_z",
    "pseudorange_m",
    "sigma_m",
    "receiver_position_x",
    "receiver_position_y",
    "receiver_position_z",
    "receiver_velocity_x",
    "receiver_velocity_y",
    "receiver_velocity_z"};

GnssStatus read_header(const char *line, int *csv_fields_index)
{
    if (!line)
        return GNSS_INVALID_ARGUMENT;

    StringSplitResult *res = string_split(line, ',');

    if (!res)
        return GNSS_INVALID_ARGUMENT;

    for (int split_index = 0; split_index < (int)res->count; split_index++)
        for (int field_index = 0; field_index < CSV_FIELD_COUNT; field_index++)
            if (strcmp(res->data[split_index], CSV_FIELD_NAMES[field_index]) == 0)
                csv_fields_index[field_index] = split_index;

    string_split_free(res);

    for (int i = 0; i < CSV_FIELD_COUNT; i++)
        if (csv_fields_index[i] == -1)
        {
            printf("Missing field %s\n", CSV_FIELD_NAMES[i]);
            return GNSS_INVALID_ARGUMENT;
        }

    return GNSS_OK;
}

GnssSimulationData *read_simulation_csv(const char *path)
{
    GnssSimulationData *data = NULL;

    FILE *stream = fopen(path, "r");
    bool is_first = true;
    int csv_fields_index[CSV_FIELD_COUNT] = {-1, -1, -1, -1, -1, -1, -1, -1};
    char raw_line[1024];
    GnssSimulationEpoch *current_epoch = NULL;
    while (fgets(raw_line, sizeof(raw_line), stream))
    {

        char *line = malloc(strlen(raw_line) + 1);
        if (line == NULL)
            return data;
        strcpy(line, raw_line);

        if (is_first)
        {
            switch (read_header(line, csv_fields_index))
            {
            case GNSS_OK:
                data = malloc(sizeof(GnssSimulationData));
                data->is_valid = true;
                data->epoch_count = 0;
                break;
            default:
                data = NULL;
                break;
            }
            is_first = false;
        }
        else
        {
            StringSplitResult *res = string_split(line, ',');
            if (!res)
                continue;

            GnssMeasurement meas;
            meas.transmission_time_s = atof(res->data[csv_fields_index[CSV_TRANSMISSION_TIME]]);
            meas.reception_time_s = atof(res->data[csv_fields_index[CSV_RECEPTION_TIME]]);
            meas.availability_time_s = meas.reception_time_s + processing_delay_s;
            meas.satellite_id = atoi(res->data[csv_fields_index[CSV_SATELLITE_ID]]);
            meas.satellite_position.x = atof(res->data[csv_fields_index[CSV_SATELLITE_X]]);
            meas.satellite_position.y = atof(res->data[csv_fields_index[CSV_SATELLITE_Y]]);
            meas.satellite_position.z = atof(res->data[csv_fields_index[CSV_SATELLITE_Z]]);
            meas.pseudorange_m = atof(res->data[csv_fields_index[CSV_PSEUDORANGE]]);
            meas.sigma_m = atof(res->data[csv_fields_index[CSV_SIGMA]]);
            meas.status = GNSS_OK;

            if (current_epoch == NULL || meas.reception_time_s != current_epoch->time_s)
            {
                if (data->epoch_count >= GNSS_SIMULATION_MAX_EPOCHS)
                    current_epoch = NULL;
                else
                {
                    current_epoch = malloc(sizeof(GnssSimulationEpoch));
                    current_epoch->measurement_count = 0;
                    current_epoch->time_s = meas.reception_time_s;
                    current_epoch->real_receiver_state.position.x = atof(res->data[csv_fields_index[CSV_RECEIVER_POSITION_X]]);
                    current_epoch->real_receiver_state.position.y = atof(res->data[csv_fields_index[CSV_RECEIVER_POSITION_Y]]);
                    current_epoch->real_receiver_state.position.z = atof(res->data[csv_fields_index[CSV_RECEIVER_POSITION_Z]]);
                    current_epoch->real_receiver_state.velocity.x = atof(res->data[csv_fields_index[CSV_RECEIVER_VELOCITY_X]]);
                    current_epoch->real_receiver_state.velocity.y = atof(res->data[csv_fields_index[CSV_RECEIVER_VELOCITY_Y]]);
                    current_epoch->real_receiver_state.velocity.z = atof(res->data[csv_fields_index[CSV_RECEIVER_VELOCITY_Z]]);
                    data->epochs[data->epoch_count] = current_epoch;
                    data->epoch_count++;
                }
            }
            if (current_epoch != NULL)
            {
                if (current_epoch->measurement_count < GNSS_SIMULATION_MAX_SATELLITES_PER_MEASUREMENT)
                {
                    current_epoch->measurements[current_epoch->measurement_count] = malloc(sizeof(GnssMeasurement));
                    *current_epoch->measurements[current_epoch->measurement_count] = meas;
                    current_epoch->measurement_count++;
                }
            }
        }

        free(line);

        if (data == NULL)
            return NULL;
    }

    printf("Done reading\n");
    printf("Epochs: %d\n", data->epoch_count);

    return data;
}