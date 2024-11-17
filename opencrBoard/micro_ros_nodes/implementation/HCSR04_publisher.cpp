#include "HCSR04_publisher.h"
#include "error_check.h"

HCSR04Publisher::HCSR04Publisher(String nodeName, HCSR04Configuration sensorConfiguration)
    : Node(nodeName),
      _ultraSonicSensor(sensorConfiguration)
{

}

void HCSR04Publisher::setup(String topic, rclc_support_t &support)
{
    RCCHECK(rclc_node_init_default(&_node, _nodeName.c_str(), "", &support ));

    rcl_node_t node = getNodeHandle();
    RCCHECK(rclc_publisher_init_default(&_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), topic.c_str()));
}

void HCSR04Publisher::publishMeasurement()
{
    float distance = _ultraSonicSensor.getMeasurement();

    _msg.header.stamp = 
}
