#include "node_core.h"
#include "micro_ros_error_check.h"

Node::Node(const char* name, rclc_support_t &support)
: _node_name(name) 
{
    setup(support);
}

void Node::setup(rclc_support_t &support)
{
    RCCHECK(rclc_node_init_default(&_node, _node_name, "", &support ));  //TODO: add checks RCC check
    initialize();
}

rcl_node_t& Node::getNodeHandle()
{
    return _node;
}

Node::~Node()
{
    rcl_node_fini(&_node);
}