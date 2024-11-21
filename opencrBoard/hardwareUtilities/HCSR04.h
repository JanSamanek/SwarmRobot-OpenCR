// ultrasonic sensor HC-SR04
#ifndef HCSR04_H
#define HCSR04_H

 struct HCSR04Configuration
 {
    int trigPin;
    int echoPin;
    int minimumRange;
    int maximumRange;
    double fieldOfView;
    String referenceFrameId;
 };


class HCSR04
{
public:
    HCSR04(const HCSR04Configuration& configuration);
    const HCSR04Configuration configuration;
    float getMeasurement();
};

#endif