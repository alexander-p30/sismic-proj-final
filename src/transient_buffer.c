#include "transient_buffer.h"

void initBuffer(TransientBuffer *buf) {
    buf->idx = 0;

    unsigned long i = 0;
    for (i = 0; i < BUFFER_SIZE; i++)
        buf->data[i] = 0;
}

void pushToBuffer(TransientBuffer *buf, uint8_t val) {
    unsigned long currentIndex = buf->idx;
    buf->data[currentIndex] = val;
    buf->idx = (currentIndex + 1) % BUFFER_SIZE;
}
