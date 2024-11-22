// ultrasonic sensor HC-SR04
#ifndef HCSR04_H
#define HCSR04_H

#include <string>
#include <sensor_msgs/msg/range.h>

 struct HCSR04Configuration
 {
    int trigPin;
    int echoPin;
    int minimumRange;
    int maximumRange;
    double fieldOfView;
    std::string referenceFrameId;
 };


class HCSR04
{
public:
    HCSR04(const HCSR04Configuration& configuration);
    const HCSR04Configuration configuration;
    float getMeasurement() const;
};

sensor_msgs__msg__Range generateMeasurementMessage(const HCSR04& ultraSonicSensor);

#endif