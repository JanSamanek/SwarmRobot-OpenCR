#include "commands.h"
#include <CAN.h>
#include "checksum.h"
#include "error_check.h"

void sendCommand(Command* command)
{
    can_message_t message;
    message.id = 0x201;
    message.format = CAN_STD_FORMAT;

    uint32_t commandLen = command->length;
    // Serial.printf("message (%i): \n", commandLen);
    uint16_t commandDataIndex = 0;
    while (commandLen > 0)
    {
        message.length = 8;
        for (int i = 0; i < 8; i++)
        {
            if (!(commandLen > 0))
            {
                message.length = i;
                break;
            }
            message.data[i] = command->data[commandDataIndex++];
            commandLen--;
            // Serial.printf("0x%02X ", message.data[i]);
        }
        CanBus.writeMessage(&message);
    }
    // Serial.println();
}


static void updateMoveCommand(Command*  command, Instructions instruction)
{
    command->data[11] = instruction.speedY & 0xFF;
    command->data[12] = ((instruction.speedX << 3) | (instruction.speedY >> 8)) & 0x07;
    command->data[13] = (instruction.speedX >> 5) & 0x3F;

    command->data[16] = (instruction.speedRotation << 4) | 0x08;
    command->data[17] = (instruction.speedRotation >> 4) & 0xFF;

    command->data[19] = 0x02 | ((instruction.speedRotation << 2) & 0xFF);
    command->data[20] = (instruction.speedRotation >> 6) & 0xFF;
}

static void updateGimballCommand(Command* command, Instructions instruction)
{
    command->data[13] = instruction.gimballRoll & 0xFF;
    command->data[14] = (instruction.gimballRoll >> 8) & 0xFF;
    command->data[15] = instruction.gimballYaw & 0xFF;
    command->data[16] = (instruction.gimballYaw >> 8) & 0xFF;
}

static void updateCommandCounter(Command* command, int command_counter)
{
    command->data[6] = command_counter & 0xFF;
    command->data[7] = (command_counter >> 8) & 0xFF;
}

static void updateCrc(Command* command, uint32_t command_length)
{
    setCRC8(command->data, 3);
    appendCRC16CheckSum(command->data, command_length);
}

static void preprocessCommand(Command* command, int command_counter)
{
    updateCommandCounter(command, command_counter);
    updateCrc(command, command->length);
}


static int commandCounter = 0;

Command buildCommand(CommandType type, Instructions instructions)
{
    Command resultCommand;

    switch (type) {
        
        case GIMBALL_COMMAND:
            resultCommand = gimballCommand;
            updateGimballCommand(&resultCommand, instructions);
            break;

        case MOVE_COMMAND:
            resultCommand = moveCommand;
            updateMoveCommand(&resultCommand, instructions);
            break;
        
        default:
            error_loop();    
    }

    preprocessCommand(&resultCommand, commandCounter++);
}

Command buildCommand(CommandType type)
{
    Command resultCommand;

    switch (type) {
        case COMMAND_1:
            resultCommand = command1;
            break;

        case COMMAND_2:
            resultCommand = command2;
            break;

        case COMMAND_3:
            resultCommand = command3;
            break;

        case COMMAND_4:
            resultCommand = command4;
            break;

        case COMMAND_5:
            resultCommand = command5;
            break;

        case BLASTER_COMMAND_1:
            resultCommand = blasterCommand1;
            break;

        case BLASTER_COMMAND_2:
            resultCommand = blasterCommand2;
            break;

        case INIT_FREE_MODE_COMMAND:
            resultCommand = initFreeModeCommand;
            break;

        case INIT_CHASSIS_ACCELERATION_COMMAND:
            resultCommand = initChasisAccelerationCommand;
            break;

        case INIT_COMMAND_1:
            resultCommand = initCommand1;
            break;

        case INIT_COMMAND_2:
            resultCommand = initCommand2;
            break;

        case INIT_COMMAND_3:
            resultCommand = initCommand3;
            break;

        case INIT_COMMAND_4:
            resultCommand = initCommand4;
            break;

        case INIT_COMMAND_5:
            resultCommand = initCommand5;
            break;
        default:
            error_loop();  
    }


    preprocessCommand(&resultCommand, commandCounter++);
    return resultCommand;
}
