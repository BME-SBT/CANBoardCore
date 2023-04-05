#include "platform/platform.h"
#include "driver/ioplex/ioplex_driver.h"
#include "platform/log.h"

u8 platform_status_last = 0;
MbedSPI PLATFORM_CAN_SPI(PLATFORM_PIN_SPI_MISO, PLATFORM_PIN_SPI_MOSI, PLATFORM_PIN_SPI_SCK);
CAN PLATFORM_CAN;
IOPlexDriver PLATFORM_IOPLEX(pio0, 0, 6, 15);

/**
 * Initialize all platform things
 */
void platform_init()
{
    // initialize platform
    SerialUSB.begin(115200);
    delay(1500);    // TODO: Remove for production, only in for minicom connection waiting
    SerialUSB.println("");
    SerialUSB.println("");
    SerialUSB.println("");
    SerialUSB.println("");
    log("Initializing platform...");

    PLATFORM_IOPLEX.setDigitNum(1);

    PLATFORM_CAN_SPI.begin();
}

void platform_preloop() {}
void platform_postloop() {}

void platform_set_status(u8 status)
{
    if (platform_status_last == 0 || !status) {
        platform_status_last = status;
        PLATFORM_IOPLEX.setDigitNum(status);
    }
}