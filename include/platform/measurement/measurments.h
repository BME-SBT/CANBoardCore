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
    explicit Measurement(u16 id, int frequency, DataType *dt): frequency(frequency), id(id), dataType(dt) {

    }

    template <typename DV>
    void set_value(DV val) {
        dataType->set_value(val);
    }

    u8 len() {
        return dataType->len();
    }

  private:
    template <int HZ>
    friend class Measurements;

    // timing info
    int frequency;

    u16 id;
    DataType *dataType;
};

template <int HZ>
class Measurements {
  public:
    explicit Measurements() {
        last_tick = micros();
    }

    void add_measurement(Measurement *measurement) {
        MeasurementHolder holder = {.measurement = measurement, .last_sent = 0};
        m_measurements.push_front(holder);
    }

    void tick(CAN &can_driver) {
        u64 current_time = micros();
        u64 elapsed = last_tick - current_time;
        u64 tick_time = 1E6 / HZ;
        if(elapsed > tick_time) {
            for (auto &meas : m_measurements) {
                auto gap = 1E6 / meas.measurement->frequency;
                if(current_time > meas.last_sent + gap) {
                    CAN_Frame frame = CAN_Frame(meas.measurement->id, meas.measurement->dataType->get_value(), meas.measurement->len());
                    can_driver.send(frame);
                    meas.last_sent = micros();
                }
            }

        }
    }

  private:
    struct MeasurementHolder {
        Measurement *measurement;
        u64 last_sent;
    };

    u64 last_tick;

    std::forward_list<MeasurementHolder> m_measurements;
};

#endif // CANBOARDCOMMON_MEASURMENTS_H
