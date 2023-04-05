#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stddef.h>
#include <optional>

template <typename T, size_t N>
class StackRingBuffer
{

public:
    StackRingBuffer()
    {
        head = buffer;
        tail = buffer;
        full = false;
        last = (buffer + N);
        empty = true;
    }

    bool is_empty()
    {
        logf("RingBuffer is empty? %d", empty);
        return empty;
    }

    bool is_full()
    {
        return full;
    }

    bool push(T value)
    {
        if (full)
        {
            return false;
        }
        *head = value;
        head = advance(head);
        if (head == tail)
        {
            full = true;
        }
        empty = false;
        return true;
    }

    T get()
    {
        if (empty)
        {
            return T{};
        }
        T val = *tail;
        tail = advance(tail);
        if (tail == head)
        {
            empty = true;
        }
        return val;
    }

    T *advance(T *ptr)
    {
        T *next = ptr++;
        if (next > last)
        {
            next = buffer;
        }
        return next;
    }

private:
    T buffer[N];
    T *head;
    T *tail;
    bool full;
    bool empty;
    const T *last;
};

#endif