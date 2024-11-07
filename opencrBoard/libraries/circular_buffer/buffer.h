#ifndef BUFFER_H
#define BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define NAMEOF_BUFFER(x) #x

typedef struct {
    uint8_t * const buffer;
    int head;                   // data read head
    int tail;                   // data write head
    const int capacity;         // maximum size of the buffer
} circ_buff_t;

#define CIRC_BUFF_DEF(x,y)                \
    uint8_t x##_data_space[y];            \
    circ_buff_t x = {                     \
        .buffer = x##_data_space,         \
        .head = 0,                        \
        .tail = 0,                        \
        .capacity = y                     \
    };     

typedef enum {
    BUFFER_READY,     
    BUFFER_EMPTY,    
    BUFFER_FULL   
} buffer_status_t;

buffer_status_t circ_buff_push(circ_buff_t *cb, uint8_t data);
buffer_status_t circ_buff_pop(circ_buff_t *cb, uint8_t *data);
void circ_buff_rollback_tail(circ_buff_t *cb, int rollback_amount);

#ifdef __cplusplus
}
#endif

#endif  // BUFFER_H