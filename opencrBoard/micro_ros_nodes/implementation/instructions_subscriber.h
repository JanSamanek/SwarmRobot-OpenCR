#ifndef INSTRUCTIONS_NODE_H
#define INSTRUCTIONS_NODE_H

#include <geometry_msgs/msg/twist.h>
#include "node_core.h"
#include "instructions.h"

class InstructionsSubscriber : public Node
{
private:
    rcl_subscription_t _subscriber;
    static Instructions _instructions;

public:
    InstructionsSubscriber(String nodeName);
    geometry_msgs__msg__Twist msg;
    void setup(String topic, rclc_support_t &support) override; 
    static void subscriptionCallback(const void* msgin);
    rcl_subscription_t& getSubscriptionHandle();
    Instructions getInstructions();
};

#endif