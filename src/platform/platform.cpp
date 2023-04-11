#include "platform/platform.h"
#include "driver/ioplex/ioplex_driver.h"
#include "platform/log.h"
#include <hardware/watchdog.h>


/*
 * Hardware driver instances
 */
CAN PLATFORM_CAN;
IOPlexDriver PLATFORM_IOPLEX(pio0, 0, 6, 15);

/**
 * Initialize all platform things
 */
UnsafeRingBuffer<PlatformStatus, 6> g_status_stack;
void platform_init() {
    // initialize platform
    SerialUSB.begin(115200);
    delay(1500); // TODO: Remove for production, only in for minicom connection
    SerialUSB.println("");
    SerialUSB.println("");
    SerialUSB.println("");
    SerialUSB.println("");

    memset(irq_logbuf, 0,LOG_IRQ_PRINTF_SIZE);
    for(int i = 0; i < 6; i++) {
        g_status_stack.add(PlatformStatus::STATUS_OK);
    }

    if(watchdog_caused_reboot()) {
        SerialUSB.println("!!!!!!!!WATCHDOG RESET!!!!!!!!");
    }


    log("Initializing platform...");
    PLATFORM_IOPLEX.setDigitNum(0x01);

    // enable watchdog reboot if not in debug mode
#ifndef DEBUG
    watchdog_enable(PLATFORM_WATCHDOG_TIMEOUT_MS, true);
#else
#pragma message("watchdog disabled in debug mode")
#endif
}

void platform_preloop() {}
void platform_postloop() {

    // print IRQ log
    afterirq_log();
    // finally, reset the watchdog
    watchdog_update();
}


PlatformStatus platform_status = PlatformStatus::STATUS_OK;

void platform_set_status(PlatformStatus status) {
    if(status != platform_status) {
        if(status == PlatformStatus::STATUS_OK) {
            g_status_stack.clear();
        }else {
            g_status_stack.add(status, true);
        }

        platform_status = status;
        PLATFORM_IOPLEX.setDigitNum(statuscode(status));
    }
}