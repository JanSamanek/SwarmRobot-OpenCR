#ifndef NODE_CORE_H
#define NODE_CORE_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>

class Node
{
protected:
    rcl_node_t _node;
    const char* _node_name;
    virtual void initialize() = 0; 
    void setup(rclc_support_t &support);
private:

public:
    Node(const char* name, rclc_support_t &support);
    virtual ~Node();
    rcl_node_t& getNodeHandle();
};

#endif