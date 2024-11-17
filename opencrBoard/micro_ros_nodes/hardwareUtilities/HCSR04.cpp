#include "HCSR04.h"
#include "Arduino.h"

HCSR04::HCSR04(const HCSR04Configuration& configuration)
:
configuration(configuration)
{
    pinMode(configuration.trigPin, OUTPUT);
    pinMode(configuration.echoPin, INPUT);
}

float HCSR04::getMeasurement()
{
    digitalWrite(configuration.trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(configuration.trigPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(configuration.trigPin, LOW);
    long duration = pulseIn(configuration.echoPin, HIGH);
  
    float distance = duration * 0.034 / 2;
    return distance;
}
