#include "error_check.h"
#include <Arduino.h>

void error_loop() {
    while (1) {
        digitalWrite(ERROR_LED_PIN, !digitalRead(ERROR_LED_PIN));
        delay(100);
    }
}