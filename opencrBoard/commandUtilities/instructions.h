#ifndef INSTRUCTIONS_H
#define INSTRUCTIONS_H

#include <stdint.h>

typedef struct {
    uint16_t speedX;
    uint16_t speedY;
    uint16_t speedRotation;
    int16_t gimballYaw;
    int16_t gimballRoll;
} Instructions;

#endif // INSTRUCTIONS_H

