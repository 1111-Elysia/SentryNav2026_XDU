#ifndef CANOPEN_PARSER_H
#define CANOPEN_PARSER_H

#include "hipnuc_can_common.h"

#ifdef __cplusplus
extern "C" {
#endif

int canopen_parse_frame(const hipnuc_can_frame_t *frame, can_sensor_data_t *data);

#ifdef __cplusplus
}
#endif

#endif
