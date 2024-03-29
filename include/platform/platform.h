#ifndef PLATFORM_H
#define PLATFORM_H

#include "../driver/can/can.h"
#include "../lib/inttypes.h"
#include "../lib/utils.h"
#include "log.h"
#include "platform/measurement/measurments.h"
#include "status.h"
// #include "measurement/measurments.h"


/*
 * Pinout, hardware config
 */
#define PLATFORM_PIN_SPI_SCK 2
#define PLATFORM_CAN_SPI_BAUD 5E6
#define PLATFORM_CAN_OSCILLATOR 16E6
#define PLATFORM_CAN_BAUD 500E3
#define PLATFORM_PIN_SPI_MOSI 3
#define PLATFORM_PIN_SPI_MISO 4
#define PLATFORM_PIN_CAN_CS 5
#define PLATFORM_PIN_CAN_INT 1


/*
 * Soft config
 */
#define PLATFORM_WATCHDOG_TIMEOUT_MS 100

extern CAN PLATFORM_CAN;
extern Measurements<100> PLATFORM_MEASUREMENTS;

void platform_init();
void platform_set_status(PlatformStatus status);

void platform_preloop();
void platform_postloop();
void platform_yield();

extern PlatformStatus platform_status;

#endif