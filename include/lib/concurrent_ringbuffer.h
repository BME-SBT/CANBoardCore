//
// Created by Barrow099 on 2023. 04. 06..
//

#ifndef CANBOARDCOMMON_CONCURRENT_RINGBUFFER_H
#define CANBOARDCOMMON_CONCURRENT_RINGBUFFER_H

#include <cstddef>
#include "lib/ringbuffer.h"
#include <pico/mutex.h>

template <typename T, size_t N> class ConcurrentRingBuffer {

  public:
    explicit ConcurrentRingBuffer(): m_lock({}) {
        mutex_init(&m_lock);
    }

    bool is_full() {
        bool ret = false;
        mutex_enter_blocking(&m_lock);
        ret = m_buffer.is_full();
        mutex_exit(&m_lock);
        return ret;
    }

    bool is_empty() {
        bool ret = false;
        mutex_enter_blocking(&m_lock);
        ret = m_buffer.is_empty();
        mutex_exit(&m_lock);
        return ret;
    }

    bool try_add(T obj, bool overwrite = false) {
        bool ret = false;
        if(!mutex_try_enter(&m_lock, nullptr)){
            return false;
        }
        ret = m_buffer.add(obj, overwrite);
        mutex_exit(&m_lock);
        return ret;
    }

    bool add(T obj, bool overwrite = false) {
        bool ret = false;
        mutex_enter_blocking(&m_lock);
        ret = m_buffer.add(obj, overwrite);
        mutex_exit(&m_lock);
        return ret;
    }

    T* peek() {
        T* p = nullptr;
        mutex_enter_blocking(&m_lock);
        p = m_buffer.peek();
        mutex_exit(&m_lock);
        return p;
    }

    bool pull(T* dest) {
        bool ret = false;
        mutex_enter_blocking(&m_lock);
        ret = m_buffer.pull(dest);
        mutex_exit(&m_lock);
        return ret;
    }

  private:
    UnsafeRingBuffer<T, N> m_buffer;
    mutex_t m_lock{};
};

#endif // CANBOARDCOMMON_CONCURRENT_RINGBUFFER_H
