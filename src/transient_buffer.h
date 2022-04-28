#ifndef SRC_TRANSIENT_BUFFER_H_
#define SRC_TRANSIENT_BUFFER_H_
#include <stdint.h>

#define BUFFER_SIZE 16

typedef struct {
    unsigned long idx;
    uint8_t data[BUFFER_SIZE];
} TransientBuffer;

void initBuffer(TransientBuffer *buf);
void pushToBuffer(TransientBuffer *buf, uint8_t val);

#endif
