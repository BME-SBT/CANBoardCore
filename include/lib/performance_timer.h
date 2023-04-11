//
// Created by Barrow099 on 2023. 04. 11..
//

#ifndef CANBOARDCOMMON_PERFORMANCE_TIMER_H
#define CANBOARDCOMMON_PERFORMANCE_TIMER_H

#include "lib/inttypes.h"
#include "Arduino.h"

class PerformanceTimer {
  public:
    static PerformanceTimer start() {
        return PerformanceTimer();
    }
  private:
    explicit PerformanceTimer() {
        start_micros = micros();
    }
  public:
    u64 elapsed() const {
        return micros() - start_micros;
    }

    u64 elapsed_millis() {
        return elapsed() / 1000;
    }

    void print() {
        u64 elapsd = elapsed();
        if(elapsd < 1000) {
            SerialUSB.print(elapsd);
            SerialUSB.print("us");
        }else if(elapsd < 1000 * 1000) {
            SerialUSB.print((float)elapsd / 1000.0);
            SerialUSB.print("ms");
        }else {
            SerialUSB.print((float)elapsd / 1000.0 / 1000.0);
            SerialUSB.print("s");
        }

    }
  private:
    u64 start_micros;
};
#endif // CANBOARDCOMMON_PERFORMANCE_TIMER_H
