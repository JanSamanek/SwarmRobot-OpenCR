#ifndef INSTRUCTIONS_H
#define INSTRUCTIONS_H

#include <stdint.h>
#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>

typedef struct {
    uint16_t speedX;
    uint16_t speedY;
    uint16_t speedRotation;
    int16_t gimballYaw;
    int16_t gimballRoll;
} Instructions;

Instructions convertToInstructions(const geometry_msgs__msg__Twist& msg);

#endif // INSTRUCTIONS_H

