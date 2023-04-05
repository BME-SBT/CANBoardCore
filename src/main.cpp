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
} g_stats;
static AppState g_state = AppState::SETUP;

/**
 * Global setup code. Runs before user setup.
 */
void global_setup() {
    // Global setup
    bool err = PLATFORM_CAN.init();
    if (err) {
        g_state = AppState::FATAL;
    }

    platform_set_status(0x00);
    g_state = AppState::USER_SETUP;
}

/**
 * User setup code. Calls the app's setup, checks for errors.
 */
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

/**
 * Main loop code. Calls the app's loop, checks errors
 */
void user_loop() {
    static u8 last_status = 0;
    static u64 last_status_print = 0;

    int err = app_loop();
    if (err) {
        platform_set_status(err);
        g_state = AppState::FATAL;
    }
    if (platform_status_last != last_status ||
        millis() > (last_status_print + 1000)) {
        logf("status: %x, loop-time: %dus", platform_status_last,
             (int)g_stats.last_loop_time);
        last_status = platform_status_last;
        last_status_print = millis();
    }
}

/**
 * Global error state. The board has crashed, waiting for watchdog restart
 */
void global_error() {
    // TODO: Log error if possible

    // This blocks as the execution cannot continue
    while (2) {
        delay(100);
        SerialUSB.print("HARD ERROR: ");
        SerialUSB.print(platform_status_last, HEX);
        SerialUSB.println();
    }
}

/**
 * Arduino setup code. Called by the framework and initializes the platform.
 */
void setup() { platform_init(); }

/**
 * This is the Arduino loop, called by the framework
 */
void loop() {
    u64 loop_start = micros();

    // Execute platform preloop steps
    platform_preloop();

    // Execute loop phase based on app state
    // These phases MUST NOT block
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

    // Execute postloop steps
    platform_postloop();

    // Calculate loop time
    u64 loop_time = micros() - loop_start;
    // TODO: Validate loop time
    g_stats.last_loop_time = loop_time;
}