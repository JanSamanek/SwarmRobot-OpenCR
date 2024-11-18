#ifndef HCSR04_PUBLISHER_NODE_H
#define HCSR04_PUBLISHER_NODE_H

#include "node_core.h"
#include "hardwareUtilities/HCSR04.h"
#include <sensor_msgs/msg/range.h>

class HCSR04Publisher : public Node
{
private:
    rcl_publisher_t _publisher;
    HCSR04 _ultraSonicSensor;

public:
    HCSR04Publisher(String nodeName, HCSR04Configuration sensorConfiguration);
    void setup(String topic, rclc_support_t &support) override; 
    void publishMeasurement();
};

#endif