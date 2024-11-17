// ultrasonic sensor HC-SR04
#ifndef HCSR04_H
#define HCSR04_H

class HCSR04Configuration
{
private:
    const int _trigPin;
    const int _echoPin;
    const int _minimumRange;
    const int _maximumRange;
    const double _fieldOfView;
    const int _referenceFrameId;

public:
    HCSR04Configuration(int minimumRange, int maximumRange, double fieldOfView, int referenceFrameId);

};

class HCSR04
{
private:
    const HCSR04Configuration _configuration;

public:
    HCSR04(const HCSR04Configuration& configuration);
    float getMeasurement();
};

#endif