//
// Created by Barrow099 on 2023. 04. 06..
//

#ifndef CANBOARDCOMMON_RINGBUFFER_H
#define CANBOARDCOMMON_RINGBUFFER_H

#include <cstddef>

/**
 * This is a FIFO implementation using a fix sized circular buffer.
 *
 * Note: This is _NOT THREAD SAFE_
 * @tparam T the type of data the buffer is storing. It has to be default contructed
 * @tparam N the number of items inside the buffer
 */
template <typename T, size_t N> class UnsafeRingBuffer {

  public:
    UnsafeRingBuffer(): m_size(0), m_head(m_data) {}

    bool is_full() {
        return m_size == N;
    }

    bool is_empty() {
        return m_size == 0;
    }

    /**
     * Add a new item into the buffer
     * @param obj the object to be added
     * @param overwrite overwrite the last item
     * @return whether the insertion was successful
     */
    bool add(T obj, bool overwrite = false) {
        bool full = is_full();
        if(!full || overwrite) {
            *m_head = obj;
            m_size = full ? m_size : (m_size + 1);
            size_t offset = m_head - m_data;
            size_t new_head = (offset + 1) % N;
            m_head = m_data + new_head;

            return !overwrite || !full;
        }
        return false;
    }

    /**
     * Peek the last element from the buffer.
     * @return pointer to the last item, nullptr if the buffer is empty.
     */
    T* peek() {
        if(is_empty()) {
            return nullptr;
        }
        return &m_data[get_tail() % N];
    }

    /**
     * Remove the last item from the list
     * @param dest Pointer to a location where the data will be placed. If nullptr,
     * the data item will be lost
     * @return whether the removal was successful.
     */
    bool pull(T* dest) {
        if(is_empty()) {
            return false;
        }

        if(dest) {
            *dest = m_data[get_tail()];
        }
        m_size--;
        return true;
    }

    void clear() {
        m_size = 0;
    }

    void get_all(T *buffer) {
        for(int i = 0; i < m_size; i++) {
            buffer[i] = m_data[((get_tail() % N) + i) % N];
        }
    }

  private:
    int get_tail() {
        int current_head = m_head - m_data;
        return (current_head + (N - m_size)) % N;
    }


    int m_size;
    T m_data[N];
    T *m_head;
};

#endif // CANBOARDCOMMON_RINGBUFFER_H
