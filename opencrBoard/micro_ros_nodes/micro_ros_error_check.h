#ifndef ERROR_CHECK_H
#define ERROR_CHECK_H

#include <Arduino.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define LED_PIN 22

inline void error_loop() {
    while (1) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}

#endif // ERROR_CHECK_H