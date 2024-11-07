#ifndef COMMAND_UPLOADING_H
#define COMMAND_UPLOADING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void upload_command_to_buffer(uint8_t* command, uint32_t command_length);

#ifdef __cplusplus
}
#endif

#endif // COMMAND_UPLOADING_H