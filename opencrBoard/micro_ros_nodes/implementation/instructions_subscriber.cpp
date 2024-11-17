#include "instructions_subscriber.h"
#include "error_check.h"

Instructions InstructionsSubscriber::_instructions = {1024, 1024, 1024, 0, 0};


InstructionsSubscriber::InstructionsSubscriber(String nodeName)
    : Node(nodeName)
{

}

void InstructionsSubscriber::setup(String topic, rclc_support_t &support)
{
    RCCHECK(rclc_node_init_default(&_node, _nodeName.c_str(), "", &support ));

    rcl_node_t node = getNodeHandle();
    RCCHECK(rclc_subscription_init_default(&_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "moveInstructions"));
}

void InstructionsSubscriber::subscriptionCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    _instructions.speedX = msg->linear.x;
    _instructions.speedY = msg->linear.y;
    _instructions.speedRotation = msg->angular.z;
}


rcl_subscription_t& InstructionsSubscriber::getSubscriptionHandle()
{
    return _subscriber;
}

Instructions InstructionsSubscriber::getInstructions()
{
    return _instructions;
}
