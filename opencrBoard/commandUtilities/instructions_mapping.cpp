#include "instructions_mapping.h"

// m/s
#define MAX_SPEED_X 3.5f 
#define MIN_SPEED_X -3.5f 

// m/s
#define MAX_SPEED_Y 3.5f 
#define MIN_SPEED_Y -3.5f

// °/s
#define MAX_SPEED_Z 600.0f
#define MIN_SPEED_Z -600.0f


#define INSTRUCTION_MAX_SPEED_X 1624
#define INSTRUCTION_MIN_SPEED_X 424 

#define INSTRUCTION_MAX_SPEED_Y 1324
#define INSTRUCTION_MIN_SPEED_Y 724

#define INSTRUCTION_MAX_SPEED_Z 1474
#define INSTRUCTION_MIN_SPEED_Z 574

Instructions convertToInstructions(const geometry_msgs__msg__Twist &msg)
{
    Instructions instructions;
    instructions.speedX = convert(msg.linear.x, MAX_SPEED_X, MIN_SPEED_X, INSTRUCTION_MAX_SPEED_X, INSTRUCTION_MIN_SPEED_X);
    instructions.speedY = convert(msg.linear.y, MAX_SPEED_Y, MIN_SPEED_Y, INSTRUCTION_MAX_SPEED_Y, INSTRUCTION_MIN_SPEED_Y);
    instructions.speedRotation = convert(msg.angular.z, MAX_SPEED_Z, MIN_SPEED_Z, INSTRUCTION_MAX_SPEED_Z, INSTRUCTION_MIN_SPEED_Z);
    return instructions;
}

static int convert(float speed, float maxSpeed, float minSpeed, int instructionMaxSpeed, int instructionMinSpeed)
{
    if(speed > maxSpeed)
    {
        return instructionMaxSpeed;
    }
    
    if (speed < minSpeed)
    {
        return instructionMinSpeed;
    }

    return instructionMinSpeed + ((speed - minSpeed) * (instructionMaxSpeed - instructionMinSpeed)) / (maxSpeed - minSpeed);
}