//
// Created by Barrow099 on 2023. 04. 08..
//

#ifndef PRIORITYAGEQUEUE_H
#define PRIORITYAGEQUEUE_H

#include <cstdint>

template <typename T, int N>
class PriorityAgeQueue {

  public:
    explicit PriorityAgeQueue() {
        for(int index = 0; index < N; index++) {
            m_priorities[index] = PriorityItem(UINT32_MAX,  &m_data[index]);
        }
    }

    bool insert(T item) {
        uint32_t prio = item.priority();

        // find leftmost location
        int index;
        for(index = 0; index < N; index++) {
            if(prio <= m_priorities[index].priority)
                break;
        }
        if(index == N) {
            return false;
        }

        // shift right, use the fallen place
        T* location = m_priorities[N-1].location;
        for(int i = N - 1; i > index; i--) {
            m_priorities[i] = m_priorities[i-1];
        }
        m_priorities[index].priority = prio;
        m_priorities[index].location = location;
        *m_priorities[index].location = item;


        return true;
    }

    T* peek() {
        if(m_priorities[0].priority == UINT32_MAX) {
            return nullptr;
        }
        for(int index = 0; index < N - 1; index++) {
            if(m_priorities[index].priority < m_priorities[index + 1].priority)
                return m_priorities[index].location;
        }

        return m_priorities[N - 1].location;
    }

    bool pull(T *dest) {
        if(m_priorities[0].priority == UINT32_MAX) {
            return false;
        }
        // find oldest lowest priority
        int position;
        for(position = 0; position < N - 1; position++) {
            if(m_priorities[position].priority < m_priorities[position + 1].priority) {
                break;
            }
        }
        if(dest) {
            *dest = *m_priorities[position].location;
        }

        T* removed_loc = m_priorities[position].location;
        for(int i = position; i < N - 1; i++) {
            m_priorities[i] = m_priorities[i+1];
        }
        m_priorities[N - 1].priority = UINT32_MAX;
        m_priorities[N - 1].location = removed_loc;

        return true;
    }

  private:
    struct PriorityItem {
        uint32_t priority;
        T* location;

        PriorityItem(uint32_t priority, T *location) : priority(priority), location(location) {}

        explicit PriorityItem() : priority(UINT32_MAX), location(nullptr) {}
    };



    // format: 0:1 0:2 0:3 1:1 2:1 3:1 3:2 ...
    PriorityItem m_priorities[N];
    T m_data[N] {};
};

#endif //PRIORITYAGEQUEUE_H
