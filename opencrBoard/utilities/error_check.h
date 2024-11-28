#ifndef ERROR_CHECK_H
#define ERROR_CHECK_H

#define ERROR_LED_PIN 22
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.println(#fn); error_loop();}}

void error_loop();

#endif // ERROR_CHECK_H