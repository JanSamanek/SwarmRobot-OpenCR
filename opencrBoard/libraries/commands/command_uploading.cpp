#include "command_uploading.h"
#include "command_preprocessing.h"
#include "buffer.h"
#include <RTOS.h>

extern circ_buff_t can_data_buff;
extern circ_buff_t can_data_len_buff;

extern SemaphoreHandle_t data_sem;
extern SemaphoreHandle_t data_len_sem;

void upload_command_to_buffer(uint8_t* command, uint32_t command_length)
{
    static uint32_t command_counter = 0;

    preprocess_command(command, command_length, command_counter);

    if(data_sem != NULL && osSemaphoreWait( data_sem, ( TickType_t ) 10 ) == osOK ) 
    {
        for (uint32_t i = 0; i < command_length; i++)
        {
            buffer_status_t status = circ_buff_push(&can_data_buff, command[i]);
            if(status == BUFFER_FULL) { }
        }
        osSemaphoreRelease(data_sem);
    }

    if(data_len_sem != NULL && osSemaphoreWait( data_len_sem, ( TickType_t ) 10 ) == osOK ) 
    {
        buffer_status_t status = circ_buff_push(&can_data_len_buff, command_length);
        if(status == BUFFER_FULL) { }
        osSemaphoreRelease(data_len_sem);
    }

    command_counter++;
}