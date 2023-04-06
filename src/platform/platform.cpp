#include "platform/platform.h"
#include "driver/ioplex/ioplex_driver.h"
#include "platform/log.h"
#include <hardware/watchdog.h>

PlatformStatus platform_status_last = PlatformStatus::STATUS_OK;
MbedSPI PLATFORM_CAN_SPI(PLATFORM_PIN_SPI_MISO, PLATFORM_PIN_SPI_MOSI,
                         PLATFORM_PIN_SPI_SCK);
CAN PLATFORM_CAN;
IOPlexDriver PLATFORM_IOPLEX(pio0, 0, 6, 15);

/**
 * Initialize all platform things
 */
void platform_init() {
    // initialize platform
    SerialUSB.begin(115200);
    delay(1500); // TODO: Remove for production, only in for minicom connection
    SerialUSB.println("");
    SerialUSB.println("");
    SerialUSB.println("");
    SerialUSB.println("");

    if(watchdog_caused_reboot()) {
        SerialUSB.println("!!!!!!!!WATCHDOG RESET!!!!!!!!");
    }


    log("Initializing platform...");
    PLATFORM_IOPLEX.setDigitNum(0x01);

    // Initialize CAN SPI
    PLATFORM_CAN_SPI.begin();

    // enable watchdog reboot if not in debug mode
#ifndef DEBUG
    watchdog_enable(PLATFORM_WATCHDOG_TIMEOUT_MS, true);
#else
#pragma message("watchdog disabled in debug mode")
#endif
}

void platform_preloop() {}
void platform_postloop() {

    // finally, reset the watchdog
    watchdog_update();
}

void platform_set_status(PlatformStatus status) {
    if (platform_status_last == PlatformStatus::STATUS_OK ||
        status != PlatformStatus::STATUS_OK) {
        platform_status_last = status;
        PLATFORM_IOPLEX.setDigitNum(static_cast<int>(status));
    }
}