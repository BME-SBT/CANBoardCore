//
// Created by Barrow099 on 2023. 04. 08..
//

#ifndef PRIORITYAGEQUEUE_H
#define PRIORITYAGEQUEUE_H

#include "can_frame.h"
#include "lib/ringbuffer.h"
#include <cstdint>

#define QUEUE_USE_SMART 1

class CANPriorityQueueRB {
  public:
    explicit CANPriorityQueueRB(int size) : m_buffer(size) {

    }

    bool insert(CAN_Frame item) {
        return m_buffer.add(item, true);
    }

    CAN_Frame *peek() {
        return m_buffer.peek();
    }

    bool pull(CAN_Frame *dest) {
        return m_buffer.pull(dest);
    }


  private:
    UnsafeAllocRingBuffer<CAN_Frame> m_buffer;
};

class CANPriorityQueueImpl {
  public:
    explicit CANPriorityQueueImpl(int size) : N(size) {
        m_priorities = new PriorityItem[N];
        m_data = new CAN_Frame[N];
        for (int index = 0; index < N; index++) {
            m_priorities[index] = PriorityItem(UINT32_MAX, &m_data[index]);
        }
    }

    virtual ~CANPriorityQueueImpl() {
        delete[] m_priorities;
        delete[] m_data;
    }


    bool insert(CAN_Frame item) {
        uint32_t prio = item.priority();

        // find leftmost location
        int index;
        for (index = 0; index < N; index++) {
            if (prio <= m_priorities[index].priority)
                break;
        }
        if (index == N) {
            return false;
        }

        // shift right, use the fallen place
        CAN_Frame *location = m_priorities[N - 1].location;
        for (int i = N - 1; i > index; i--) {
            m_priorities[i] = m_priorities[i - 1];
        }
        m_priorities[index].priority = prio;
        m_priorities[index].location = location;
        *m_priorities[index].location = item;

        return true;
    }

    CAN_Frame *peek() {
        if (m_priorities[0].priority == UINT32_MAX) {
            return nullptr;
        }
        for (int index = 0; index < N - 1; index++) {
            if (m_priorities[index].priority < m_priorities[index + 1].priority)
                return m_priorities[index].location;
        }

        return m_priorities[N - 1].location;
    }

    bool pull(CAN_Frame *dest) {
        if (m_priorities[0].priority == UINT32_MAX) {
            return false;
        }
        // find oldest lowest priority
        int position;
        for (position = 0; position < N - 1; position++) {
            if (m_priorities[position].priority <
                m_priorities[position + 1].priority) {
                break;
            }
        }
        if (dest) {
            *dest = *m_priorities[position].location;
        }

        CAN_Frame *removed_loc = m_priorities[position].location;
        for (int i = position; i < N - 1; i++) {
            m_priorities[i] = m_priorities[i + 1];
        }
        m_priorities[N - 1].priority = UINT32_MAX;
        m_priorities[N - 1].location = removed_loc;

        return true;
    }

  private:
    struct PriorityItem {
        uint32_t priority;
        CAN_Frame *location;

        PriorityItem(uint32_t priority, CAN_Frame *location)
            : priority(priority), location(location) {}

        explicit PriorityItem() : priority(UINT32_MAX), location(nullptr) {}
    };

    // format: 0:1 0:2 0:3 1:1 2:1 3:1 3:2 ...
    PriorityItem *m_priorities;
    CAN_Frame *m_data;
    int N;
};

class CANPriorityQueue {
  public:
    explicit CANPriorityQueue(int size) : impl(size) {}

    bool pull(CAN_Frame *dest) {
        return impl.pull(dest);
    }

    CAN_Frame *peek() {
        return impl.peek();
    }

    bool insert(CAN_Frame item) {
        return impl.insert(item);
    }

  private:
#if QUEUE_USE_SMART
    CANPriorityQueueImpl impl;
#else
    CANPriorityQueueRB impl;
#endif
};

#endif // PRIORITYAGEQUEUE_H
