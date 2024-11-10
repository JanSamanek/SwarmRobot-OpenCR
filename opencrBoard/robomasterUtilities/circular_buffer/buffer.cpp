#include "buffer.h"                                 

buffer_status_t circ_buff_push(circ_buff_t *cb, uint8_t data)
{
    int next;

    next = cb->head + 1;        // next is where head will point to after this write
    if (next >= cb->capacity)
        next = 0;

    if (next == cb->tail)           // buffer full (if head + 1 == tail)
        return BUFFER_FULL;

    cb->buffer[cb->head] = data;    // load data and move
    cb->head = next;                // head to next data offset
    return BUFFER_READY;            // return success
};

buffer_status_t circ_buff_pop(circ_buff_t *cb, uint8_t *data)
{
    int next;

    if (cb->head == cb->tail)       // buffer empty
        return BUFFER_EMPTY;

    next = cb->tail + 1;            // next is where tail will point to after this read
    if(next >= cb->capacity)
        next = 0;

    *data = cb->buffer[cb->tail];   // read data and move
    cb->tail = next;                // tail to next data offset
    return BUFFER_READY;            // return success
    
};

void circ_buff_rollback_tail(circ_buff_t *cb, int rollback_amount)
{
    cb->tail = (cb->tail - rollback_amount + cb->capacity) % cb->capacity;
}
