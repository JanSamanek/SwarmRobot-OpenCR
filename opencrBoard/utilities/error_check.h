#ifndef ERROR_CHECK_H
#define ERROR_CHECK_H

#define ERROR_LED_PIN 22
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = (fn); \
    if (temp_rc != RCL_RET_OK) { \
      Serial.printf("Error: %s failed with error code %d\n", #fn, temp_rc); \
      error_loop(); \
      return; \
    } \
  }

void error_loop();

#endif // ERROR_CHECK_H