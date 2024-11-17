#include "node_core.h"
#include "error_check.h"

int Node::_nodeCounter = 0;

Node::Node(String name, rclc_support_t &support)
: _nodeName(name + "_" + String(_nodeCounter)) 
{
    setup(support);
    _nodeCounter++;
}

void Node::setup(rclc_support_t &support)
{
    RCCHECK(rclc_node_init_default(&_node, _nodeName.c_str(), "", &support ));
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