//
// Created by Barrow099 on 2023. 04. 06..
//

#ifndef CANBOARDCOMMON_CONCURRENT_RINGBUFFER_H
#define CANBOARDCOMMON_CONCURRENT_RINGBUFFER_H

#include "lib/ringbuffer.h"
#include "ringbuffer.h"
#include <cstddef>
#include <pico/critical_section.h>

/**
 * IRQ and thread-safe RingBuffer
 * @tparam T
 * @tparam N
 */
template <typename T, size_t N> class ConcurrentRingBuffer {

  public:
    explicit ConcurrentRingBuffer(): m_lock({}) {
        critical_section_init(&m_lock);
    }

    bool is_full() {
        bool ret;
        critical_section_enter_blocking(&m_lock);
        ret = m_buffer.is_full();
        critical_section_exit(&m_lock);
        return ret;
    }

    bool is_empty() {
        bool ret;
        critical_section_enter_blocking(&m_lock);
        ret = m_buffer.is_empty();
        critical_section_exit(&m_lock);
        return ret;
    }

    bool add(T obj, bool overwrite = false) {
        bool ret;
        critical_section_enter_blocking(&m_lock);
        ret = m_buffer.add(obj, overwrite);
        critical_section_exit(&m_lock);
        return ret;
    }

    T* peek() {
        T* p;
        critical_section_enter_blocking(&m_lock);
        p = m_buffer.peek();
        critical_section_exit(&m_lock);
        return p;
    }

    bool pull(T* dest) {
        bool ret ;
        critical_section_enter_blocking(&m_lock);
        ret = m_buffer.pull(dest);
        critical_section_exit(&m_lock);
        return ret;
    }

  private:
    UnsafeRingBuffer<T, N> m_buffer;
    critical_section_t m_lock{};
};

#endif // CANBOARDCOMMON_CONCURRENT_RINGBUFFER_H
