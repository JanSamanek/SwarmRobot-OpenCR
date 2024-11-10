#ifndef COMMAND_PREPROCESSING_H
#define COMMAND_PREPROCESSING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void preprocess_command(uint8_t* command, uint32_t command_length, int command_counter);

#ifdef __cplusplus
}
#endif

#endif // COMMAND_PREPROCESSING_H