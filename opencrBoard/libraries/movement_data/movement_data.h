#ifndef MOVEMENT_DATA_H
#define MOVEMENT_DATA_H

#include <stdint.h>

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
    int16_t yaw;
    int16_t roll;
} movement_data_t;

#endif // MOVEMENT_DATA_H

