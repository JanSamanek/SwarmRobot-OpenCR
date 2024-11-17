#include "node_core.h"
#include "error_check.h"

int Node::_nodeCounter = 0;

Node::Node(String name)
: _nodeName(name) 
{

}

rcl_node_t& Node::getNodeHandle()
{
    return _node;
}

Node::~Node()
{
    rcl_node_fini(&_node);
}