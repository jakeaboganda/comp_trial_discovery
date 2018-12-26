
#ifndef __RINGBUF32__
#define __RINGBUF32__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

typedef uint32_t RINGBUF32_DATA;

typedef volatile struct {
    volatile RINGBUF32_DATA *data;
    uint32_t mask;

    uint32_t start;
    uint32_t end;
} RINGBUF32;

static inline bool ringbuf32_initialize(
        RINGBUF32 *r,
        volatile RINGBUF32_DATA *buffer,
        uint32_t size);

static inline uint32_t ringbuf32_put(
        RINGBUF32 *r,
        volatile  RINGBUF32_DATA *data,
        uint32_t dataLength);

static inline uint32_t ringbuf32_putLong(
        RINGBUF32 *r,
        RINGBUF32_DATA data);

static inline uint32_t ringbuf32_get(
        RINGBUF32 *r,
        volatile RINGBUF32_DATA *output_data_buffer,
        uint32_t output_data_buffer_size);

static inline uint32_t ringbuf32_getLong(
        RINGBUF32 *r,
        volatile RINGBUF32_DATA *output_data_buffer);

static inline int ringbuf32_size(RINGBUF32 *r);

static inline int ringbuf32_elements(RINGBUF32 *r);



#define ringbuf32_isPowerofTwo(x) (((x) & ((x) - 1)) == 0)
#define ringbuf32_dataInBuffer(r) ((r->start - r->end) & r->mask)

static inline bool ringbuf32_initialize(
        RINGBUF32 *r,
        volatile RINGBUF32_DATA *buffer,
        uint32_t size)
{
    if(r == NULL || buffer == NULL || !ringbuf32_isPowerofTwo(size))
    {
        return false;
    }

    r->data = buffer;
    r->mask = size - 1;
    r->start = 0;
    r->end = 0;

    return true;
}

static inline uint32_t ringbuf32_put(
        RINGBUF32 *r,
        volatile RINGBUF32_DATA *data,
        uint32_t dataLength)
{
    // TODO: make this more efficient
    uint32_t i;
    for(i = 0; i < dataLength; i++)
    {
        // Check if full
        if(ringbuf32_dataInBuffer(r) == r->mask)
        {
            break;
        }
        r->data[r->start] = data[i];
        r->start = (r->start + 1) & r->mask;
    }

    return i;
}

static inline uint32_t ringbuf32_putLong(
        RINGBUF32 *r,
        RINGBUF32_DATA data)
{
    // Check if full
    if(ringbuf32_dataInBuffer(r) == r->mask)
    {
        return 0;
    }
    r->data[r->start] = data;
    r->start = (r->start + 1) & r->mask;

    return 1;
}

static inline uint32_t ringbuf32_get(
        RINGBUF32 *r,
        volatile RINGBUF32_DATA *output_data_buffer,
        uint32_t output_data_buffer_size)
{
    // TODO: Make this more efficient

    uint32_t i;
    for(i = 0; i < output_data_buffer_size; i++)
    {
        if(ringbuf32_dataInBuffer(r) <= 0)
        {
            break;
        }
        output_data_buffer[i] = r->data[r->end];
        r->end = (r->end + 1) & r->mask;
    }
    return i;
}

static inline uint32_t ringbuf32_getLong(
        RINGBUF32 *r,
        volatile RINGBUF32_DATA *output_data_buffer)
{
    if(ringbuf32_dataInBuffer(r) <= 0)
    {
        return 0;
    }

    output_data_buffer[0] = r->data[r->end];
    r->end = (r->end + 1) & r->mask;

    return 1;
}

static inline int ringbuf32_size(RINGBUF32 *r)
{
    if(r == NULL)
    {
        return -1;
    }

    return (int)(r->mask + 1);
}

static inline int ringbuf32_elements(RINGBUF32 *r)
{
    if(r == NULL)
    {
        return -1;
    }

    return ringbuf32_dataInBuffer(r);
}

#endif // __RINGBUF32__
