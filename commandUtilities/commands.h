#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>
#include "instructions.h"

#define MAX_COMMAND_SIZE 73

typedef enum
{
    COMMAND_1,
    COMMAND_2,
    COMMAND_3,
    COMMAND_4,
    COMMAND_5,
    GIMBALL_COMMAND,
    MOVE_COMMAND,
    BLASTER_COMMAND_1,
    BLASTER_COMMAND_2,
    INIT_FREE_MODE_COMMAND,
    INIT_CHASSIS_ACCELERATION_COMMAND,
    INIT_COMMAND_1,
    INIT_COMMAND_2,
    INIT_COMMAND_3,
    INIT_COMMAND_4,
    INIT_COMMAND_5
} CommandType;

typedef struct {
    uint8_t data[MAX_COMMAND_SIZE];   
    uint16_t length;   
    CommandType type;    
} Command;

void sendCommand(Command* command);

Command buildCommand(CommandType type, Instructions instructions);
Command buildCommand(CommandType type);

extern Command command1;
extern Command command2;
extern Command command3;
extern Command command4;
extern Command command5;
extern Command gimballCommand;
extern Command moveCommand;
extern Command blasterCommand1;
extern Command blasterCommand2;
extern Command initFreeModeCommand;
extern Command initChasisAccelerationCommand;
extern Command initCommand1;
extern Command initCommand2;
extern Command initCommand3;
extern Command initCommand4;
extern Command initCommand5;

#endif // COMMANDS_H