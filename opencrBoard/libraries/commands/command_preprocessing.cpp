#include "command_preprocessing.h"
#include "checksum.h"
#include "commands.h"
#include "movement_data.h"

extern movement_data_t movement_data;

static void update_movement_data(uint8_t* command)
{
    command[11] = movement_data.y & 0xFF;
    command[12] = ((movement_data.x << 3) | (movement_data.y >> 8)) & 0x07;
    command[13] = (movement_data.x >> 5) & 0x3F;

    command[16] = (movement_data.z << 4) | 0x08;
    command[17] = (movement_data.z >> 4) & 0xFF;

    command[19] = 0x02 | ((movement_data.z << 2) & 0xFF);
    command[20] = (movement_data.z >> 6) & 0xFF;
}

static void update_gimbal_data(uint8_t* command)
{
    command[13] = movement_data.roll & 0xFF;
    command[14] = (movement_data.roll >> 8) & 0xFF;
    command[15] = movement_data.yaw & 0xFF;
    command[16] = (movement_data.yaw >> 8) & 0xFF;
}

static void update_command_counter(uint8_t* command, int command_counter)
{
    command[6] = command_counter & 0xFF;
    command[7] = (command_counter >> 8) & 0xFF;
}

static void update_crc(uint8_t* command, uint32_t command_length)
{
    setCRC8(command, 3);
    appendCRC16CheckSum(command, command_length);
}

void preprocess_command(uint8_t* command, uint32_t command_length, int command_counter)
{
     if(command == gimball_command)
     {
        update_gimbal_data(command);
     }
            
     if(command == move_command)
     {
        update_movement_data(command);
     }

    update_command_counter(command, command_counter);
    update_crc(command, command_length);
}