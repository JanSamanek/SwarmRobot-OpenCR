#ifndef COMMAND_FACTORY_H
#define COMMAND_FACTORY_H

#include <stdint.h>
#include "commands.h"
#include "instructions.h"


class CommandFactory
{
private:
    static uint32_t commandCounter;

    Command command1;
    Command command2;
    Command command3;
    Command command4;
    Command command5;
    Command gimballCommand;
    Command moveCommand;
    Command blasterCommand1;
    Command blasterCommand2;
    Command initFreeModeCommand;
    Command initChasisAccelerationCommand;
    Command initCommand1;
    Command initCommand2;
    Command initCommand3;
    Command initCommand4;
    Command initCommand5;


public:
    CommandFactory();
    ~CommandFactory();
    Command buildCommand(CommandType type, const Instructions& instructions) const;
    Command buildCommand(CommandType type) const;
};

#endif // COMMAND_FACTORY_H