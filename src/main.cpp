#include "app.h"
#include "platform/log.h"
#include "platform/platform.h"
#include <Arduino.h>

enum class AppState {
    SETUP,
    USER_SETUP,
    LOOP,
    FATAL,
};

static struct {
    u64 last_loop_time;
} stats;

static AppState g_state = AppState::SETUP;

void setup() { platform_init(); }

void global_setup() {
    // Global setup
    bool err = PLATFORM_CAN.init();
    if (err) {
        g_state = AppState::FATAL;
    }

    platform_set_status(0x00);
    g_state = AppState::USER_SETUP;
}

void user_setup() {
    // User setup
    int err = app_setup();
    if (err) {
        platform_set_status(err);
        g_state = AppState::FATAL;
    } else {
        g_state = AppState::LOOP;
    }
}

u8 last_status = 0;
u64 last_status_print = 0;
void user_loop() {
    int err = app_loop();
    if (err) {
        platform_set_status(err);
        g_state = AppState::FATAL;
    }
    if (platform_status_last != last_status ||
        millis() > (last_status_print + 1000)) {
        logf("status: %x, loop-time: %dus", platform_status_last,
             (int)stats.last_loop_time);
        last_status = platform_status_last;
        last_status_print = millis();
    }
}

void global_error() {
    // TODO: Log error if possible

    while (2) {
        delay(100);
        SerialUSB.print("HARD ERROR: ");
        SerialUSB.print(platform_status_last, HEX);
        SerialUSB.println();
    }
}

void loop() {
    u64 start = micros();
    platform_preloop();
    switch (g_state) {
    case AppState::SETUP:
        global_setup();
        break;
    case AppState::USER_SETUP:
        user_setup();
        break;
    case AppState::LOOP:
        user_loop();
        break;
    case AppState::FATAL:
        global_error();
        break;
    }
    platform_postloop();
    u64 loop_time = micros() - start;
    stats.last_loop_time = loop_time;
}