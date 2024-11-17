#ifndef NODE_CORE_H
#define NODE_CORE_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>

class Node
{
protected:
    rcl_node_t _node;
    const String _nodeName;
    virtual void setup(String topic, rclc_support_t &support) = 0; 

private:
    static int _nodeCounter;

public:
    Node(String name);
    virtual ~Node();
    rcl_node_t& getNodeHandle();
};

#endif