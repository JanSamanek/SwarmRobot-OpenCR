#include <RTOS.h>

#include "buffer.h"
#include "movement_data.h"
#include "commands.h"
#include "command_uploading.h"
#include "can_bus.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3.h>

#define BOOT_TIMEOUT 5000
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

geometry_msgs__msg__Vector3 msg;
rcl_subscription_t subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

CIRC_BUFF_DEF(can_data_buff, 2048)  
CIRC_BUFF_DEF(can_data_len_buff, 2048)  

osSemaphoreId data_sem;
osSemaphoreId data_len_sem; 
osSemaphoreId movement_data_sem; 

movement_data_t movement_data = 
{
  .x = 1024,
  .y = 1024,
  .z = 1024,
  .yaw = 0,
  .roll = 0
};

#define ERROR_LED_PIN 22
void error_loop(){
  while(1){
    digitalWrite(ERROR_LED_PIN, !digitalRead(ERROR_LED_PIN));
    delay(100);
  }
}

void setup() 
{
  set_microros_transports();

  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LED_PIN, HIGH);  

  osDelay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "movement_data_topic"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
  Serial.begin(115200);

  osSemaphoreDef(DATA_SEMAPHORE);      
  osSemaphoreDef(DATA_LEN_SEMAPHORE); 
  osSemaphoreDef(MOVEMENT_DATA_SEMAPHORE);  

  data_sem = osSemaphoreCreate(osSemaphore(DATA_SEMAPHORE), 1);
  data_len_sem = osSemaphoreCreate(osSemaphore(DATA_LEN_SEMAPHORE), 1);
  movement_data_sem = osSemaphoreCreate(osSemaphore(MOVEMENT_DATA_SEMAPHORE), 1);

  osSemaphoreRelease(data_sem);
  osSemaphoreRelease(data_len_sem);
  osSemaphoreRelease(movement_data_sem);

  upload_command_to_buffer(init_free_mode_command, init_free_mode_command[1]);
  upload_command_to_buffer(init_chasis_acc_on_command, init_chasis_acc_on_command[1]);
  upload_command_to_buffer(init_command_1, init_command_1[1]);
  upload_command_to_buffer(init_command_2, init_command_2[1]);
  upload_command_to_buffer(init_command_3, init_command_3[1]);
  upload_command_to_buffer(init_command_4, init_command_4[1]);
  upload_command_to_buffer(init_command_5, init_command_5[1]);

  osThreadDef(THREAD_NAME_SUBSCRIBER, subscriber_thread, osPriorityNormal, 0, 8196);
  osThreadCreate(osThread(THREAD_NAME_SUBSCRIBER), NULL);

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

static void subscriber_thread(void const *argument)
{
  (void) argument;

  while (1) 
  {
    osDelay(1);
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  }
}

void subscription_callback(const void * msgin)
{  
  const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;
  if(movement_data_sem != NULL && osSemaphoreWait( movement_data_sem, ( TickType_t ) 10 ) == osOK ) 
  {
    movement_data.x = msg->x;
    movement_data.y = msg->y;
    movement_data.z = msg->z;
    osSemaphoreRelease(movement_data_sem);
  } 
}

