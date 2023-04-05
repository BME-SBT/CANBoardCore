#include "platform/platform.h"
#include "platform/log.h"

u8 platform_status_last = 0;
MbedSPI PLATFORM_CAN_SPI(PLATFORM_PIN_SPI_MISO, PLATFORM_PIN_SPI_MOSI, PLATFORM_PIN_SPI_SCK);
CAN PLATFORM_CAN;

void platform_init()
{
    // initialize platform
    SerialUSB.begin(115200);
    delay(2000);
    SerialUSB.println("");
    SerialUSB.println("");
    SerialUSB.println("");
    SerialUSB.println("");
    log("Initializing platform...");
    PLATFORM_CAN_SPI.begin();
}

void platform_preloop() {}
void platform_postloop() {}

void platform_set_status(u8 status)
{
    if (platform_status_last == 0 || !status)
        platform_status_last = status;
}