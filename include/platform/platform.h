#ifndef PLATFORM_H
#define PLATFORM_H

#include <Arduino.h>
#include <SPI.h>
#include "../lib/utils.h"
#include "../lib/inttypes.h"
#include "status.h"
#include "log.h"
#include "../driver/can/can.h"

/*
 * Pinout, hardware config
 */
#define PLATFORM_PIN_SPI_SCK 2
#define PLATFORM_CAN_SPI_BAUD 1E6    // 1 MHz
#define PLATFORM_CAN_OSCILLATOR 16E6 // 16 MHz
#define PLATFORM_CAN_BAUD 500E3      // 500 kHZ
#define PLATFORM_PIN_SPI_MOSI 3
#define PLATFORM_PIN_SPI_MISO 4
#define PLATFORM_PIN_CAN_CS 5
#define PLATFORM_PIN_CAN_INT 1


/*
 * Soft config
 */
#define PLATFORM_WATCHDOG_TIMEOUT_MS 100

extern MbedSPI PLATFORM_CAN_SPI;
extern CAN PLATFORM_CAN;

void platform_init();
void platform_set_status(PlatformStatus status);

void platform_preloop();
void platform_postloop();

extern PlatformStatus platform_status_last;

#endif