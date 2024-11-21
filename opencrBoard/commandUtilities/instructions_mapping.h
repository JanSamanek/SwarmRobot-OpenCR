#ifndef INSTRUCTIONS_MAPPING_H
#define INSTRUCTIONS_MAPPING_H
    
#include "instructions.h"
#include <geometry_msgs/msg/twist.h>

Instructions convertToInstructions(const geometry_msgs__msg__Twist& msg);

#endif