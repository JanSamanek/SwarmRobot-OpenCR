#include "node_core.h"

int Node::_nodeCounter = 0;

Node::Node(std::string name)
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