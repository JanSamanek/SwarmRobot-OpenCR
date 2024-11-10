#include <CAN.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void init_can();
void send_command_from_buffer_over_CAN();

#ifdef __cplusplus
}
#endif
