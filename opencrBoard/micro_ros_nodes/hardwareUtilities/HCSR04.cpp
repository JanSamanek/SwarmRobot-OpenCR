#include "HCSR04.h"
#include "Arduino.h"

HCSR04::HCSR04(int trigPin, int echoPin)
:
_trigPin(trigPin),
_echoPin(echoPin)
{
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float HCSR04::getMeasurement()
{
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(_trigPin, LOW);
    long duration = pulseIn(_echoPin, HIGH);
  
    float distance = duration * 0.034 / 2;
    return distance;
}
