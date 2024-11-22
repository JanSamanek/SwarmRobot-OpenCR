#include "commands.h"
#include "buffer.h"
#include "instructions.h"
#include "command_factory.h"
#include "HCSR04.h"

#include <CAN.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/range.h>

#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>

#include "error_check.h"

rcl_subscription_t instructionsSubscriber;
geometry_msgs__msg__Twist instructionMsg;

rcl_publisher_t publisher;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

CircularBuffer buffer(2048);
CommandFactory factory;

#define BOOT_TIMEOUT 5000
#define PERIOD_1000_MS 1000000
#define PERIOD_100_MS 100000
#define PERIOD_10_MS 10000

HardwareTimer Timer1000ms(TIMER_CH1);
HardwareTimer Timer100ms(TIMER_CH2);
HardwareTimer Timer10ms(TIMER_CH3);

Instructions instructions = {1024, 1024, 1024, 0, 0};


void publishMeasurement(HCSR04 ultraSonicSensor)
{
    float distance = ultraSonicSensor.getMeasurement();
    int64_t epoch_time_ns = rmw_uros_epoch_nanos();

    sensor_msgs__msg__Range msg;

    msg.header.frame_id = micro_ros_string_utilities_set(msg.header.frame_id, ultraSonicSensor.configuration.referenceFrameId.c_str());
    msg.header.stamp.sec = epoch_time_ns / 1000000000;
    msg.header.stamp.nanosec = epoch_time_ns % 1000000000;
    msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    msg.field_of_view = ultraSonicSensor.configuration.fieldOfView * (M_PI / 180);
    msg.min_range = ultraSonicSensor.configuration.minimumRange;
    msg.max_range = ultraSonicSensor.configuration.maximumRange;
    msg.range = distance;

    RCCHECK(rcl_publish(&publisher, &msg, NULL));
}


void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  instructions = convertToInstructions(*msg);
}

void setup() 
{   
  set_microros_transports();

  Serial.begin(115200);
  CanBus.begin(CAN_BAUD_1000K, CAN_STD_FORMAT);
  
  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LED_PIN, HIGH);  

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &instructionsSubscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "instructions"));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "HCSRO4/measurement"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &instructionsSubscriber, &instructionMsg, &subscription_callback, ON_NEW_DATA));


  Timer1000ms.stop();
  Timer1000ms.setPeriod(PERIOD_1000_MS);        
  Timer1000ms.attachInterrupt(callback_1000_ms);

  Timer100ms.stop();
  Timer100ms.setPeriod(PERIOD_100_MS);        
  Timer100ms.attachInterrupt(callback_100_ms);

  Timer10ms.stop();
  Timer10ms.setPeriod(PERIOD_10_MS);        
  Timer10ms.attachInterrupt(callback_10_ms);

  buffer.push(factory.buildCommand(INIT_FREE_MODE_COMMAND));
  buffer.push(factory.buildCommand(INIT_CHASSIS_ACCELERATION_COMMAND));
  buffer.push(factory.buildCommand(INIT_COMMAND_1));
  buffer.push(factory.buildCommand(INIT_COMMAND_2));
  buffer.push(factory.buildCommand(INIT_COMMAND_3));
  buffer.push(factory.buildCommand(INIT_COMMAND_4));
  buffer.push(factory.buildCommand(INIT_COMMAND_5));
  
  delay(BOOT_TIMEOUT); 

  Timer1000ms.start();
  Timer100ms.start();
  Timer10ms.start();
}

void loop() 
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

void callback_1000_ms(void)
{
  buffer.push(factory.buildCommand(COMMAND_4));
  buffer.push(factory.buildCommand(COMMAND_5));
}

void callback_100_ms(void)
{
  buffer.push(factory.buildCommand(COMMAND_1));
  buffer.push(factory.buildCommand(COMMAND_2));
  buffer.push(factory.buildCommand(COMMAND_3));
}

void callback_10_ms(void)
{
  buffer.push(factory.buildCommand(MOVE_COMMAND, instructions));
  buffer.push(factory.buildCommand(GIMBALL_COMMAND, instructions));
}