#include <RTOS.h>
#include "buffer.h"
#include "movement_data.h"
#include "commands.h"
#include "command_uploading.h"
#include "can_bus.h"

#define BOOT_TIMEOUT 5000

CIRC_BUFF_DEF(can_data_buff, 2048)  
CIRC_BUFF_DEF(can_data_len_buff, 2048)  

osSemaphoreId data_sem;
osSemaphoreId data_len_sem; 

movement_data_t movement_data = 
{
  .x = 1324,
  .y = 1024,
  .z = 1024,
  .yaw = 0,
  .roll = 0
};

void setup() 
{
  Serial.begin(115200);

  osSemaphoreDef(DATA_SEMAPHORE);      
  osSemaphoreDef(DATA_LEN_SEMAPHORE);   

  data_sem = osSemaphoreCreate(osSemaphore(DATA_SEMAPHORE), 1);
  data_len_sem = osSemaphoreCreate(osSemaphore(DATA_LEN_SEMAPHORE), 1);

  osSemaphoreRelease(data_sem);
  osSemaphoreRelease(data_len_sem);

  upload_command_to_buffer(init_free_mode_command, init_free_mode_command[1]);
  upload_command_to_buffer(init_chasis_acc_on_command, init_chasis_acc_on_command[1]);
  upload_command_to_buffer(init_command_1, init_command_1[1]);
  upload_command_to_buffer(init_command_2, init_command_2[1]);
  upload_command_to_buffer(init_command_3, init_command_3[1]);
  upload_command_to_buffer(init_command_4, init_command_4[1]);
  upload_command_to_buffer(init_command_5, init_command_5[1]);


  osThreadDef(THREAD_NAME_CAN, can_thread, osPriorityNormal, 0, 8196);
  osThreadCreate(osThread(THREAD_NAME_CAN), NULL);

  osKernelStart();
}

void loop() 
{

}

void timer_1000ms_callback(const void* argument)
{
  upload_command_to_buffer(command_4, command_4[1]);  
  upload_command_to_buffer(command_5, command_5[1]);
}

void timer_100ms_callback(const void* argument)
{
  upload_command_to_buffer(command_1, command_1[1]);  
  upload_command_to_buffer(command_2, command_2[1]);  
  upload_command_to_buffer(command_3, command_3[1]);
}

void timer_10ms_callback(const void *argument)
{
  upload_command_to_buffer(move_command, move_command[1]);
  // upload_command_to_buffer(gimball_command, gimball_command[1]);
}

static void can_thread(void const *argument)
{
  (void) argument;

  osDelay(BOOT_TIMEOUT); 
  init_can();

  osTimerDef(TIMER_10ms, timer_10ms_callback);
  osTimerId timer_10ms_id = osTimerCreate(osTimer(TIMER_10ms), osTimerPeriodic, NULL);

  osTimerDef(TIMER_100ms, timer_100ms_callback);
  osTimerId timer_100ms_id = osTimerCreate(osTimer(TIMER_100ms), osTimerPeriodic, NULL);

  osTimerDef(TIMER_1000ms, timer_1000ms_callback);
  osTimerId timer_1000ms_id = osTimerCreate(osTimer(TIMER_1000ms), osTimerPeriodic, NULL);

  osTimerStart(timer_1000ms_id, 1000);
  osTimerStart(timer_100ms_id, 100);
  osTimerStart(timer_10ms_id, 10);

  for(;;)
  {
    send_command_from_buffer_over_CAN();
    osDelay(1);
  }
}