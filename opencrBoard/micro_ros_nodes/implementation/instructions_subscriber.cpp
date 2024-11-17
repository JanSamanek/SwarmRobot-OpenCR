#include "instructions_subscriber.h"
#include "error_check.h"

Instructions InstructionsSubscriber::_instructions = {1024, 1024, 1024, 0, 0};

InstructionsSubscriber::InstructionsSubscriber(rclc_support_t &support) 
: 
Node("instructions_subscriber_node", support)
{

}

void InstructionsSubscriber::subscriptionCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    _instructions.speedX = msg->linear.x;
    _instructions.speedY = msg->linear.y;
    _instructions.speedRotation = msg->angular.z;
}

void InstructionsSubscriber::initialize()
{
    rcl_node_t node = getNodeHandle();
    RCCHECK(rclc_subscription_init_default(&_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "microRos/moveInstructions"));
}

rcl_subscription_t& InstructionsSubscriber::getSubscriptionHandle()
{
    return _subscriber;
}

Instructions InstructionsSubscriber::getInstructions()
{
    return _instructions;
}
