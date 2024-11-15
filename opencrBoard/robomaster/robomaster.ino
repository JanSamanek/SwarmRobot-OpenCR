#include "commands.h"
#include "buffer.h"
#include "instruction.h"
#include "command_factory.h"
#include <CAN.h>

#define BOOT_TIMEOUT 5000

CircularBuffer buffer(2048);
CommandFactory factory;

MovementInstruction movementData = 
{
  .speedX = 1024,
  .speedY = 1024,
  .rotation = 1024,
  .gimballYaw = 0,
  .gimballRoll = 0
};

#define PERIOD_1000_MS 1000000
#define PERIOD_100_MS 100000
#define PERIOD_10_MS 10000

HardwareTimer Timer1000ms(TIMER_CH1);
HardwareTimer Timer100ms(TIMER_CH2);
HardwareTimer Timer10ms(TIMER_CH3);


void setup() 
{   
  (BOOT_TIMEOUT); 

  CanBus.begin(CAN_BAUD_1000K, CAN_STD_FORMAT);
  Serial.begin(115200);

  Timer1000ms.stop();
  Timer1000ms.setPeriod(PERIOD_1000_MS);        
  Timer1000ms.attachInterrupt(callback_1000_ms);

  Timer100ms.stop();
  Timer100ms.setPeriod(PERIOD_100_MS);        
  Timer100ms.attachInterrupt(callback_100_ms);

  Timer10ms.stop();
  Timer10ms.setPeriod(PERIOD_10_MS);        
  Timer10ms.attachInterrupt(callback_10_ms);

  buffer.push(factory.buildCommand(INIT_FREE_MODE_COMMAND, movementData));
  buffer.push(factory.buildCommand(INIT_CHASSIS_ACCELERATION_COMMAND, movementData));
  buffer.push(factory.buildCommand(INIT_COMMAND_1, movementData));
  buffer.push(factory.buildCommand(INIT_COMMAND_2, movementData));
  buffer.push(factory.buildCommand(INIT_COMMAND_3, movementData));
  buffer.push(factory.buildCommand(INIT_COMMAND_4, movementData));
  buffer.push(factory.buildCommand(INIT_COMMAND_5, movementData));

  Timer1000ms.start();
  Timer100ms.start();
  Timer10ms.start();
}

void loop() 
{
  Command command;

  BufferStatus status = buffer.pop(command);
  if (status == BUFFER_EMPTY)
  {
      return;
  }

  sendCommand(command);
  delay(1);
}

void callback_1000_ms(void)
{
  buffer.push(factory.buildCommand(COMMAND_4, movementData));
  buffer.push(factory.buildCommand(COMMAND_5, movementData));
}

void callback_100_ms(void)
{
  buffer.push(factory.buildCommand(COMMAND_1, movementData));
  buffer.push(factory.buildCommand(COMMAND_2, movementData));
  buffer.push(factory.buildCommand(COMMAND_3, movementData));
}

void callback_10_ms(void)
{
  buffer.push(factory.buildCommand(MOVE_COMMAND, movementData));
  buffer.push(factory.buildCommand(GIMBALL_COMMAND, movementData));
}

