#include "instructions_subscriber.h"
#include "error_check.h"
#include "instructions.h"

Instructions InstructionsSubscriber::_instructions = {1024, 1024, 1024, 0, 0};


InstructionsSubscriber::InstructionsSubscriber(std::string nodeName)
: Node(nodeName)
{

}

void InstructionsSubscriber::setup(std::string topic, rclc_support_t &support)
{
    RCCHECK(rclc_node_init_default(&_node, _nodeName.c_str(), "", &support));
    RCCHECK(rclc_subscription_init_default(&_subscriber, &_node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), topic.c_str()));
}

void InstructionsSubscriber::subscriptionCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    _instructions = convertToInstructions(*msg);
}


rcl_subscription_t& InstructionsSubscriber::getSubscriptionHandle()
{
    return _subscriber;
}

Instructions& InstructionsSubscriber::getInstructionsHandle()
{
    return _instructions;
}
