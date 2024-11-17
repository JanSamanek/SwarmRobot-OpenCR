#include "HCSR04_publisher.h"
#include "error_check.h"

HCSR04Publisher::HCSR04Publisher(HCSR04Configuration sensorConfiguration, rclc_support_t &support)
:
Node("HCSR04_measurement_publisher_node", support),
_ultraSonicSensor(sensorConfiguration)
{
}

void HCSR04Publisher::publishMeasurement()
{
}

void HCSR04Publisher::initialize()
{
    rcl_node_t node = getNodeHandle();
    RCCHECK(rclc_publisher_init_default(&_publisher, 
    &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),  
    "HCSR04/measurement"));
}
