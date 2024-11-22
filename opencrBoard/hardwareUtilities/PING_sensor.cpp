#include "PING_sensor.h"
#include "Arduino.h"
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>

PINGSensor::PINGSensor(const PINGSensorConfiguration& configuration)
:
configuration(configuration)
{

}

float PINGSensor::getMeasurement() const
{
    int pingPin = configuration.pingPin;
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin, LOW);

    pinMode(pingPin, INPUT);
    duration = pulseIn(pingPin, HIGH);
  
    float distance = duration * 0.034 / 2;
    return distance;
}

sensor_msgs__msg__Range generateMeasurementMessage(const PINGSensor &ultraSonicSensor)
{
    float distance = ultraSonicSensor.getMeasurement();
    int64_t epoch_time_ns = rmw_uros_epoch_nanos();

    sensor_msgs__msg__Range msg;

    msg.header.frame_id = micro_ros_string_utilities_set(msg.header.frame_id, ultraSonicSensor.configuration.referenceFrameId.c_str());
    msg.header.stamp.sec = epoch_time_ns / 1000000000;
    msg.header.stamp.nanosec = epoch_time_ns % 1000000000;
    msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    msg.field_of_view = ultraSonicSensor.configuration.fieldOfView * (M_PI / 180);
    msg.min_range = ultraSonicSensor.configuration.minimumRange;
    msg.max_range = ultraSonicSensor.configuration.maximumRange;
    msg.range = distance;

    return msg;
}
