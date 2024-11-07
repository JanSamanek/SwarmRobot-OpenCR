#ifndef __ROBOMASTER_S1_CRC_H__
#define __ROBOMASTER_S1_CRC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void setCRC8(uint8_t *msg, uint32_t crc_index);
void appendCRC16CheckSum(uint8_t *msg, uint32_t msglen);

#ifdef __cplusplus
}
#endif

#endif
