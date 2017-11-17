#include <ringbuf.h>

uint8_t ringbuf_init(RingBuf *rb, uint32_t size) {
	rb->data = (uint8_t*) malloc(size);
	if (!rb->data) {
		return 1;
	}

	rb->size = size;
	rb->head = size - 1;
	rb->tail = 0;
	rb->count = 0;

	return 0;
}

void ringbuf_free(RingBuf *rb) {
    if (rb->data) {
        free(rb->data);
        rb->data = NULL;
    }
}

uint8_t *ringbuf_head(RingBuf *rb) {
	return rb->data + rb->head;
}

uint8_t *ringbuf_tail(RingBuf *rb) {
	return rb->data + rb->tail;
}

uint32_t ringbuf_count(const RingBuf *rb) {
	return rb->count;
}

uint8_t ringbuf_empty(const RingBuf *rb) {
	return rb->count <= 0;
}

uint8_t ringbuf_full(const RingBuf *rb) {
	return rb->count >= rb->size;
}

uint8_t ringbuf_push(RingBuf *rb, const uint8_t *data, uint32_t size) {
	if (rb->count + size > rb->size) {
		return 1;
	}

    while (size > 0) {
        rb->head = (rb->head + 1) % rb->size;
        rb->count += 1;
        *ringbuf_head(rb) = *data;
        data += 1;
        size -= 1;
    }

	return 0;
}

uint8_t ringbuf_pop(RingBuf *rb, uint8_t *data, uint32_t size) {
	if (rb->count < size) {
		return 1;
	}

    while(size > 0) {
        *data = *ringbuf_tail(rb);
        rb->tail = (rb->tail + 1) % rb->size;
        rb->count -= 1;
        data += 1;
        size -= 1;
    }

	return 0;
}
