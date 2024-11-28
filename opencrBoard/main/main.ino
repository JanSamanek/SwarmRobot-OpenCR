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

#define PERIOD_1000_MS 1000000
#define PERIOD_100_MS 100000
#define PERIOD_10_MS 10000

HardwareTimer Timer1000ms(TIMER_CH1);
HardwareTimer Timer100ms(TIMER_CH2);
HardwareTimer Timer10ms(TIMER_CH3);

void initCommunication()
{
  Serial.begin(115200);
  CanBus.begin(CAN_BAUD_1000K, CAN_STD_FORMAT);
}

void initMicroRos()
{
  set_microros_transports();

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
    &backUltraSonicSensorPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    BACK_SENSOR_TOPIC));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &instructionsSubscriber, &instructionMsg, &incomming_instructions_callback, ON_NEW_DATA));
}

void initThreads()
{
  osThreadDef(COMMAND_HANDLER_THREAD, command_handler_thread, osPriorityNormal, 0, 8196);
  osThreadCreate(osThread(COMMAND_HANDLER_THREAD), NULL);

  osThreadDef(FRONT_DISTANCE_PUBLISHER_THREAD, front_distance_publisher_thread, osPriorityNormal, 0, 8196);
  osThreadCreate(osThread(FRONT_DISTANCE_PUBLISHER_THREAD), NULL);

  osThreadDef(BACK_DISTANCE_PUBLISHER_THREAD, back_distance_publisher_thread, osPriorityNormal, 0, 8196);
  osThreadCreate(osThread(BACK_DISTANCE_PUBLISHER_THREAD), NULL);
}

void initCommands()
{
  buffer.push(factory.buildCommand(INIT_FREE_MODE_COMMAND));
  buffer.push(factory.buildCommand(INIT_CHASSIS_ACCELERATION_COMMAND));
  buffer.push(factory.buildCommand(INIT_COMMAND_1));
  buffer.push(factory.buildCommand(INIT_COMMAND_2));
  buffer.push(factory.buildCommand(INIT_COMMAND_3));
  buffer.push(factory.buildCommand(INIT_COMMAND_4));
  buffer.push(factory.buildCommand(INIT_COMMAND_5));
}

void setup() 
{   
  pinMode(ERROR_LED_PIN, OUTPUT); 
  digitalWrite(ERROR_LED_PIN, HIGH);  

  initCommunication();
  
  set_microros_transports();

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
    &backUltraSonicSensorPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    BACK_SENSOR_TOPIC));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &instructionsSubscriber, &instructionMsg, &incomming_instructions_callback, ON_NEW_DATA));

  initCommands();
  initThreads();

  osDelay(BOOT_TIMEOUT); 
  osKernelStart();
}

void loop() 
{

}

void callback_1000ms(void)
{
  buffer.push(factory.buildCommand(COMMAND_4));
  buffer.push(factory.buildCommand(COMMAND_5));
}

void callback_100ms(void)
{
  buffer.push(factory.buildCommand(COMMAND_1));
  buffer.push(factory.buildCommand(COMMAND_2));
  buffer.push(factory.buildCommand(COMMAND_3));
}

void callback_10ms(void)
{
  buffer.push(factory.buildCommand(MOVE_COMMAND, instructions));
  buffer.push(factory.buildCommand(GIMBALL_COMMAND, instructions));
}

void incomming_instructions_callback(const void *msgin) 
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  instructions = convertToInstructions(*msg);
}

void setupTimer(HardwareTimer &timer, uint32_t period, void (*callback)(void))
{
  timer.stop();
  timer.setPeriod(period);
  timer.attachInterrupt(callback);
  timer.start();
}

void command_handler_thread(void const *argument)
{
  (void) argument;

  setupTimer(Timer1000ms, PERIOD_1000_MS, callback_1000ms);
  setupTimer(Timer100ms, PERIOD_100_MS, callback_100ms);
  setupTimer(Timer10ms, PERIOD_10_MS, callback_10ms);

  while(1)
  {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

    Command command;
    BufferStatus status = buffer.pop(command);
    if (status == BUFFER_EMPTY)
    {
      return;
    }

    sendCommand(command);
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