// ultrasonic range finder
#ifndef PING_SENSOR_H
#define PING_SENSOR_H

#include <string>
#include <sensor_msgs/msg/range.h>

 struct PINGSensorConfiguration
 {
    int pingPin;
    float minimumRange;
    float maximumRange;
    double fieldOfView;
    std::string referenceFrameId;
 };


class PINGSensor
{
public:
    PINGSensor(const PINGSensorConfiguration& configuration);
    const PINGSensorConfiguration configuration;
    float getMeasurement() const;
};

sensor_msgs__msg__Range generateMeasurementMessage(const PINGSensor& ultraSonicSensor);

#endif