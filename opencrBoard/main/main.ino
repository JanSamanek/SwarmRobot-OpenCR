#include "commands.h"
#include "buffer.h"
#include "instructions.h"
#include "command_factory.h"

#include <CAN.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "error_check.h"

#include "instructions_subscriber.h"

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

CircularBuffer buffer(2048);
CommandFactory factory;

#define BOOT_TIMEOUT 5000
#define PERIOD_1000_MS 1000000
#define PERIOD_100_MS 100000
#define PERIOD_10_MS 10000

HardwareTimer Timer1000ms(TIMER_CH1);
HardwareTimer Timer100ms(TIMER_CH2);
HardwareTimer Timer10ms(TIMER_CH3);

// Instructions instructions;

void setup() 
{   
  set_microros_transports();

  Serial.begin(115200);
  CanBus.begin(CAN_BAUD_1000K, CAN_STD_FORMAT);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  


  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  InstructionsSubscriber instructionsSubscriber("instructions_subscriber_node");
  instructionsSubscriber.setup("instructions", support);

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  rcl_subscription_t instructionsSubscription = instructionsSubscriber.getSubscriptionHandle();
  geometry_msgs__msg__Twist msg = instructionsSubscriber.msg;
  rclc_subscription_callback_t callback = InstructionsSubscriber::subscriptionCallback;
  RCCHECK(rclc_executor_add_subscription(&executor, &instructionsSubscription, &msg, callback, ON_NEW_DATA));
  
  // instructionsHandle = instructionsSubscriber.getInstructionsHandle();

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
  // buffer.push(factory.buildCommand(MOVE_COMMAND, instructionsHandle));
  // buffer.push(factory.buildCommand(GIMBALL_COMMAND, instructionsHandle));
}