
#ifndef __RINGBUF__
#define __RINGBUF__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

typedef volatile struct {
    volatile uint8_t *data;
    uint32_t mask;

    uint32_t start;
    uint32_t end;
} RINGBUF;

static inline bool ringbuf_initialize(
        RINGBUF *r,
        volatile uint8_t *buffer,
        uint32_t size);

static inline uint32_t ringbuf_put(
        RINGBUF *r,
        volatile  uint8_t *data,
        uint32_t dataLength);

static inline uint32_t ringbuf_putByte(
        RINGBUF *r,
        uint8_t data);

static inline uint32_t ringbuf_get(
        RINGBUF *r,
        volatile uint8_t *output_data_buffer,
        uint32_t output_data_buffer_size);

static inline uint32_t ringbuf_getByte(
        RINGBUF *r,
        volatile uint8_t *output_data_buffer);

static inline int ringbuf_size(RINGBUF *r);

static inline int ringbuf_elements(RINGBUF *r);



#define ringbuf_isPowerofTwo(x) (((x) & ((x) - 1)) == 0)
#define ringbuf_dataInBuffer(r) ((r->start - r->end) & r->mask)

static inline bool ringbuf_initialize(
        RINGBUF *r,
        volatile uint8_t *buffer,
        uint32_t size)
{
    if(r == NULL || buffer == NULL || !ringbuf_isPowerofTwo(size))
    {
        return false;
    }

    r->data = buffer;
    r->mask = size - 1;
    r->start = 0;
    r->end = 0;

    return true;
}

static inline uint32_t ringbuf_put(
        RINGBUF *r,
        volatile uint8_t *data,
        uint32_t dataLength)
{
    // TODO: make this more efficient
    uint32_t i;
    for(i = 0; i < dataLength; i++)
    {
        // Check if full
        if(ringbuf_dataInBuffer(r) == r->mask)
        {
            break;
        }
        r->data[r->start] = data[i];
        r->start = (r->start + 1) & r->mask;
    }

    return i;
}

static inline uint32_t ringbuf_putByte(
        RINGBUF *r,
        uint8_t data)
{
    // Check if full
    if(ringbuf_dataInBuffer(r) == r->mask)
    {
        return 0;
    }
    r->data[r->start] = data;
    r->start = (r->start + 1) & r->mask;

    return 1;
}

static inline uint32_t ringbuf_get(
        RINGBUF *r,
        volatile uint8_t *output_data_buffer,
        uint32_t output_data_buffer_size)
{
    // TODO: Make this more efficient

    uint32_t i;
    for(i = 0; i < output_data_buffer_size; i++)
    {
        if(ringbuf_dataInBuffer(r) <= 0)
        {
            break;
        }
        output_data_buffer[i] = r->data[r->end];
        r->end = (r->end + 1) & r->mask;
    }
    return i;
}

static inline uint32_t ringbuf_getByte(
        RINGBUF *r,
        volatile uint8_t *output_data_buffer)
{
    if(ringbuf_dataInBuffer(r) <= 0)
    {
        return 0;
    }

    output_data_buffer[0] = r->data[r->end];
    r->end = (r->end + 1) & r->mask;

    return 1;
}

static inline int ringbuf_size(RINGBUF *r)
{
    if(r == NULL)
    {
        return -1;
    }

    return (int)(r->mask + 1);
}

static inline int ringbuf_elements(RINGBUF *r)
{
    if(r == NULL)
    {
        return -1;
    }

    return ringbuf_dataInBuffer(r);
}

#endif // __RINGBUF__
