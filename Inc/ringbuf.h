#pragma once

#include <stdint.h>

typedef struct {
    uint8_t *data;
    uint32_t size;
    uint32_t head;
    uint32_t tail;
    uint32_t count;
} RingBuf;

uint8_t ringbuf_init(RingBuf *rb, uint32_t size);
void ringbuf_free(RingBuf *rb);
uint8_t *ringbuf_head(RingBuf *rb);
uint8_t *ringbuf_tail(RingBuf *rb);
uint32_t ringbuf_count(const RingBuf *rb);
uint8_t ringbuf_empty(const RingBuf *rb);
uint8_t ringbuf_full(const RingBuf *rb);
uint8_t ringbuf_push(RingBuf *rb, const uint8_t *data, uint32_t size);
uint8_t ringbuf_pop(RingBuf *rb, uint8_t *data, uint32_t size);
