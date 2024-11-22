// #include "HCSR04_publisher.h"
// #include "error_check.h"
// #include <rmw_microros/rmw_microros.h>
// #include <micro_ros_utilities/string_utilities.h>


// HCSR04Publisher::HCSR04Publisher(std::string nodeName, HCSR04Configuration sensorConfiguration)
//     : Node(nodeName),
//       _ultraSonicSensor(sensorConfiguration)
// {

// }

// void HCSR04Publisher::setup(std::string topic, rclc_support_t &support)
// {
//     RCCHECK(rclc_node_init_default(&_node, _nodeName.c_str(), "", &support ));

//     rcl_node_t node = getNodeHandle();
//     RCCHECK(rclc_publisher_init_default(&_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), topic.c_str()));
// }

// void HCSR04Publisher::publishMeasurement()
// {
//     float distance = _ultraSonicSensor.getMeasurement();
//     int64_t epoch_time_ns = rmw_uros_epoch_nanos();

//     sensor_msgs__msg__Range msg;

//     msg.header.frame_id = micro_ros_string_utilities_set(msg.header.frame_id, _ultraSonicSensor.configuration.referenceFrameId.c_str());
//     msg.header.stamp.sec = epoch_time_ns / 1000000000;
//     msg.header.stamp.nanosec = epoch_time_ns % 1000000000;
//     msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
//     msg.field_of_view = _ultraSonicSensor.configuration.fieldOfView * (M_PI / 180);
//     msg.min_range = _ultraSonicSensor.configuration.minimumRange;
//     msg.max_range = _ultraSonicSensor.configuration.maximumRange;
//     msg.range = distance;

//     RCCHECK(rcl_publish(&_publisher, &msg, NULL));
// }
