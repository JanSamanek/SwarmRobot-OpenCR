#include "commands.h"
#include "buffer.h"
#include "instructions.h"
#include "command_factory.h"
#include "PING_sensor.h"

#include <CAN.h>
#include <RTOS.h>

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/range.h>

#include "error_check.h"

#define BOOT_TIMEOUT 5000

#define NODE_NAME "micro_ros_arduino_node"
#define INSTRUCTIONS_TOPIC "instructions"
#define FRONT_SENSOR_TOPIC "PING/front/measurement"
#define BACK_SENSOR_TOPIC "PING/back/measurement"

rcl_publisher_t frontUltraSonicSensorPublisher;
rcl_publisher_t backUltraSonicSensorPublisher;

rcl_subscription_t instructionsSubscriber;
geometry_msgs__msg__Twist instructionMsg;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

CircularBuffer buffer(2048);
CommandFactory factory;
Instructions instructions = {1024, 1024, 1024, 0, 0};

PINGSensorConfiguration frontSensorConfig = createFrontSensorConfig();
PINGSensor frontUltraSonicSensor(frontSensorConfig);

PINGSensorConfiguration backSensorConfig = createBackSensorConfig();
PINGSensor backUltraSonicSensor(backSensorConfig);

osSemaphoreId bufferSemaphore;
osSemaphoreId instructionsSemaphore; 


void setup() 
{   
  // communication init
  Serial.begin(115200);
  CanBus.begin(CAN_BAUD_1000K, CAN_STD_FORMAT);
  
  // micro-ros init
  set_microros_transports();

  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LED_PIN, HIGH);  

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  RCCHECK(rclc_subscription_init_default(
    &instructionsSubscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    INSTRUCTIONS_TOPIC));

  RCCHECK(rclc_publisher_init_default(
    &frontUltraSonicSensorPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    FRONT_SENSOR_TOPIC));

  RCCHECK(rclc_publisher_init_default(
    &frontUltraSonicSensorPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    BACK_SENSOR_TOPIC));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &instructionsSubscriber, &instructionMsg, &incomming_instructions_callback, ON_NEW_DATA));

  // semaphore init
  osSemaphoreDef(BUFFER_SEMAPHORE);      
  osSemaphoreDef(INSTRUCTIONS_SEMAPHORE);  

  bufferSemaphore = osSemaphoreCreate(osSemaphore(BUFFER_SEMAPHORE), 1);
  instructionsSemaphore = osSemaphoreCreate(osSemaphore(INSTRUCTIONS_SEMAPHORE), 1);

  // tasks init
  osThreadDef(TIMERS_INIT_THREAD, timers_init_thread, osPriorityNormal, 0, 8196);
  osThreadCreate(osThread(TIMERS_INIT_THREAD), NULL);

  osThreadDef(SUBSCRIBER_THREAD, instructions_subscriber_thread, osPriorityNormal, 0, 8196);
  osThreadCreate(osThread(SUBSCRIBER_THREAD), NULL);

  osThreadDef(FRONT_DISTANCE_PUBLISHER_THREAD, front_distance_publisher_thread, osPriorityNormal, 0, 8196);
  osThreadCreate(osThread(FRONT_DISTANCE_PUBLISHER_THREAD), NULL);

  osThreadDef(BACK_DISTANCE_PUBLISHER_THREAD, back_distance_publisher_thread, osPriorityNormal, 0, 8196);
  osThreadCreate(osThread(BACK_DISTANCE_PUBLISHER_THREAD), NULL);

  osThreadDef(SEND_COMMAND_THREAD, send_command_thread, osPriorityNormal, 0, 8196);
  osThreadCreate(osThread(SEND_COMMAND_THREAD), NULL);
  
  // init commands
  buffer.push(factory.buildCommand(INIT_FREE_MODE_COMMAND));
  buffer.push(factory.buildCommand(INIT_CHASSIS_ACCELERATION_COMMAND));
  buffer.push(factory.buildCommand(INIT_COMMAND_1));
  buffer.push(factory.buildCommand(INIT_COMMAND_2));
  buffer.push(factory.buildCommand(INIT_COMMAND_3));
  buffer.push(factory.buildCommand(INIT_COMMAND_4));
  buffer.push(factory.buildCommand(INIT_COMMAND_5));
  
  osDelay(BOOT_TIMEOUT); 
  osKernelStart();
}

void loop() 
{

}

void callback_1000ms(const void* argument)
{
  if(bufferSemaphore != NULL) 
  {
    if(osSemaphoreWait(bufferSemaphore, ( TickType_t ) 10 ) == osOK)
    {
      buffer.push(factory.buildCommand(COMMAND_4));
      buffer.push(factory.buildCommand(COMMAND_5));
      osSemaphoreRelease(bufferSemaphore);
    }
  }
}

void callback_100ms(const void* argument)
{
  if(bufferSemaphore != NULL) 
  {
    if(osSemaphoreWait(bufferSemaphore, ( TickType_t ) 10 ) == osOK)
    {
      buffer.push(factory.buildCommand(COMMAND_1));
      buffer.push(factory.buildCommand(COMMAND_2));
      buffer.push(factory.buildCommand(COMMAND_3));
      osSemaphoreRelease(bufferSemaphore);
    }
  }
}

void callback_10ms(const void* argument)
{
  if(instructionsSemaphore != NULL) 
  {
    if(osSemaphoreWait( instructionsSemaphore, ( TickType_t ) 10 ) == osOK )
    {
      if(osSemaphoreWait(bufferSemaphore, ( TickType_t ) 10 ) == osOK)
      {
        buffer.push(factory.buildCommand(MOVE_COMMAND, instructions));
        buffer.push(factory.buildCommand(GIMBALL_COMMAND, instructions));
        osSemaphoreRelease(bufferSemaphore);
      }
      osSemaphoreRelease(instructionsSemaphore);
    }
  }
}

void timers_init_thread(void const *argument)
{
  (void) argument;

  osTimerDef(TIMER_10ms, callback_10ms);
  osTimerId timerId10ms = osTimerCreate(osTimer(TIMER_10ms), osTimerPeriodic, NULL);

  osTimerDef(TIMER_100ms, callback_100ms);
  osTimerId timerId100ms = osTimerCreate(osTimer(TIMER_100ms), osTimerPeriodic, NULL);

  osTimerDef(TIMER_1000ms, callback_1000ms);
  osTimerId timerId1000ms = osTimerCreate(osTimer(TIMER_1000ms), osTimerPeriodic, NULL);

  osTimerStart(timerId1000ms, 1000);
  osTimerStart(timerId100ms, 100);
  osTimerStart(timerId10ms, 10);
}

void incomming_instructions_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  if(instructionsSemaphore != NULL) 
  {
    if(osSemaphoreWait( instructionsSemaphore, ( TickType_t ) 10 ) == osOK )
    {
      instructions = convertToInstructions(*msg);
      osSemaphoreRelease(instructionsSemaphore);
    }
  } 
}

void instructions_subscriber_thread(void const *argument)
{
  (void) argument;

  while(1)
  {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  }
}

void front_distance_publisher_thread(void const *argument)
{
  (void) argument;

  while(1)
  {
    sensor_msgs__msg__Range msg = generateMeasurementMessage(frontUltraSonicSensor);
    RCCHECK(rcl_publish(&frontUltraSonicSensorPublisher, &msg, NULL));
  }
}

void back_distance_publisher_thread(void const *argument)
{
  (void) argument;

  while(1)
  {
    sensor_msgs__msg__Range msg = generateMeasurementMessage(backUltraSonicSensor);
    RCCHECK(rcl_publish(&backUltraSonicSensorPublisher, &msg, NULL));
  }
}

void send_command_thread(void const *argument)
{
  (void) argument;

  while(1)
  {
    if(bufferSemaphore != NULL) 
    {
      if(osSemaphoreWait(bufferSemaphore, ( TickType_t ) 10 ) == osOK)
      {
        Command command;

        BufferStatus status = buffer.pop(command);
        if (status == BUFFER_EMPTY)
        {
          return;
        }

        sendCommand(command);
        //TODO: maybe have to introduce delay to release the semaphore
        osSemaphoreRelease(bufferSemaphore);
      }
    }
  }
}
