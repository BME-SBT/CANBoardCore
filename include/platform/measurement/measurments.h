//
// Created by Barrow099 on 2023. 04. 16..
//

#ifndef CANBOARDCOMMON_MEASURMENTS_H
#define CANBOARDCOMMON_MEASURMENTS_H

#include <algorithm>
#include <api/Common.h>
#include <forward_list>

#include "datatypes.h"


class Measurement {

  public:
    explicit Measurement(u16 id, int frequency): frequency(frequency), id(id) {

    }

    void set_value(u64 val) {
        this->value = val;
    }


  private:
    template <int HZ>
    friend class Measurements;

    int frequency;
    u16 id;
    u64 value;
};

template <int HZ>
class Measurements {
  public:
    explicit Measurements() {
        last_tick = micros();
    }

    void add_measurement(Measurement measurement) {
        MeasurementHolder holder = {.measurement = measurement, .last_sent = 0};
        m_measurements.push_front(holder);
    }

    void send_frames(CAN &can_driver) {
        u64 current_time = micros();
        u64 elapsed = last_tick - current_time;
        u64 tick_time = 1000000 / HZ;
        if(elapsed > tick_time) {
            for (const auto &item : m_measurements) {
                if(item.last_sent + tick_time < )
            }

        }
    }

  private:
    struct MeasurementHolder {
        Measurement measurement;
        u64 last_sent;
    };

    u64 last_tick;

    std::forward_list<MeasurementHolder> m_measurements;
};

#endif // CANBOARDCOMMON_MEASURMENTS_H
