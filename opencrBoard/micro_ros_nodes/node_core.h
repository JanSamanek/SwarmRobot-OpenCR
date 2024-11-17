#ifndef NODE_CORE_H
#define NODE_CORE_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>

class Node
{
protected:
    rcl_node_t _node;
    const String _nodeName;
    virtual void initialize() = 0; 
    void setup(rclc_support_t &support);
private:
    static int _nodeCounter;
public:
    Node(String name, rclc_support_t &support);
    virtual ~Node();
    rcl_node_t& getNodeHandle();
};

#endif