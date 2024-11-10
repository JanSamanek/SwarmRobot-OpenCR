#include "can_bus.h"
#include "buffer.h"
#include <RTOS.h>

extern circ_buff_t can_data_buff;
extern circ_buff_t can_data_len_buff;

extern SemaphoreHandle_t data_sem;
extern SemaphoreHandle_t data_len_sem;

void init_can()
{
    if (CanBus.begin(CAN_BAUD_1000K, CAN_STD_FORMAT) == false)
    {
        Serial.println("CAN open fail!!");
    }
    else
    {
        Serial.println("CAN open success!");
    }
}

void send_command_from_buffer_over_CAN()
{
    if (data_len_sem != NULL)
    {
        if (osSemaphoreWait(data_len_sem, (TickType_t) 10) == osOK)
        {
            can_message_t message;
            message.id = 0x201;
            message.format = CAN_STD_FORMAT;

            uint8_t can_command_len = 0;

            buffer_status_t status = circ_buff_pop(&can_data_len_buff, &can_command_len);
            if (status == BUFFER_EMPTY)
            {
                osSemaphoreRelease(data_len_sem);
                return;
            }

            // Serial.printf("message (%i): \n", can_command_len);
            while (can_command_len > 0)
            {
                message.length = 8;
                for (int i = 0; i < 8; i++)
                {
                    if (!(can_command_len > 0))
                    {
                        message.length = i;
                        break;
                    }
                    buffer_status_t status = circ_buff_pop(&can_data_buff, &message.data[i]);
                    can_command_len--;
                    // Serial.printf("0x%02X ", message.data[i]);
                }
                CanBus.writeMessage(&message);
            }
            Serial.println();
            osSemaphoreRelease(data_len_sem);
        }
    }
}
