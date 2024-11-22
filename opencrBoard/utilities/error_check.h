#ifndef ERROR_CHECK_H
#define ERROR_CHECK_H

#define LED_PIN 22
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop();

#endif // ERROR_CHECK_H