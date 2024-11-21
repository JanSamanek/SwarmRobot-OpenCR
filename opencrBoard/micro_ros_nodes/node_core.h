#ifndef NODE_CORE_H
#define NODE_CORE_H

#include <micro_ros_arduino.h>
#include <rclc/rclc.h>
#include <string>

class Node
{
protected:
    rcl_node_t _node;
    const std::string _nodeName;
    virtual void setup(std::string topic, rclc_support_t &support) = 0; 

private:
    static int _nodeCounter;

public:
    Node(std::string name);
    virtual ~Node();
    rcl_node_t& getNodeHandle();
};

#endif