#ifndef HCSR04_PUBLISHER_NODE_H
#define HCSR04_PUBLISHER_NODE_H

#include "node_core.h"
#include "hardwareUtilities/HCSR04.h"
#include <sensor_msgs/msg/range.h>

class HCSR04Publisher : public Node
{
private:
    rcl_publisher_t _publisher;
    sensor_msgs__msg__Range _msg;
    void initialize() override; 
    HCSR04 _ultraSonicSensor;

public:
    HCSR04Publisher(HCSR04Configuration sensorConfiguration, rclc_support_t &support);
    void publishMeasurement();
};

#endif