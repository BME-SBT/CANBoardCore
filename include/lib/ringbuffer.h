//
// Created by Barrow099 on 2023. 04. 06..
//

#ifndef CANBOARDCOMMON_RINGBUFFER_H
#define CANBOARDCOMMON_RINGBUFFER_H

#include <cstddef>

template <typename T, size_t N> class UnsafeRingBuffer {

  public:
    UnsafeRingBuffer(): m_size(0), m_head(m_data) {}

    bool is_full() {
        return m_size == N;
    }

    bool is_empty() {
        return m_size == 0;
    }

    bool add(T obj, bool overwrite = false) {
        bool full = is_full();
        if(!full || overwrite) {
           *m_head = obj;
           m_size = full ? m_size : (m_size + 1);
           size_t offset = m_head - m_data;
           size_t new_head = (offset + 1) % N;
           m_head = m_data + new_head;

           return true;
        }
        return false;
    }

    T* peek() {
        if(is_empty()) {
           return nullptr;
        }
        return m_data[(get_tail() + m_size) % N];
    }

    bool pull(T* dest) {
        if(is_empty()) {
           return false;
        }

        *dest = m_data[get_tail()];
        m_size--;
        return true;
    }

  private:
    size_t get_tail() {
        size_t current_head = m_head - m_data;
        return (current_head + (N - m_size)) % N;
    }


    size_t m_size;
    T m_data[N];
    T *m_head;
};

#endif // CANBOARDCOMMON_RINGBUFFER_H
