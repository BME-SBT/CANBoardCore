#include "app.h"
#include "platform/log.h"
#include "platform/platform.h"

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
    bool err = true;
    while(err) {
        log("initializing CAN");
        err =  PLATFORM_CAN.init();
        delay(1000);
    }

    platform_set_status(PlatformStatus::STATUS_OK);
    g_state = AppState::USER_SETUP;
}

/**
 * User setup code. Calls the app's setup, checks for errors.
 */
void user_setup() {
    // User setup
    PlatformStatus err = app_setup();
    if (is_err(err)) {
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
    static PlatformStatus last_status = PlatformStatus::STATUS_OK;
    static u64 last_status_print = 0;

    PlatformStatus err = app_loop();
    if (is_err(err)) {
        platform_set_status(err);
        g_state = AppState::FATAL;
    }
    if (platform_status != last_status ||
        millis() > (last_status_print + 1000)) {
        logf("status: %x, loop-time: %dus", statuscode(platform_status),
             (int)g_stats.last_loop_time);
        last_status = platform_status;
        last_status_print = millis();
    }
}


extern UnsafeRingBuffer<PlatformStatus, 6> g_status_stack;
/**
 * Global error state. The board has crashed, waiting for watchdog restart
 */
[[noreturn]] void global_error() {
    // TODO: Log error if possible

    PlatformStatus status_stack[6];
    memset(status_stack, 0, sizeof(PlatformStatus) * 6);
    g_status_stack.get_all(status_stack);
    // This blocks as the execution cannot continue
    while (true) {
        delay(1000);
        SerialUSB.print("\n\nHARD ERROR: ");
        SerialUSB.print(statuscode(platform_status), HEX);
        SerialUSB.print("(");
        SerialUSB.print(statusname(platform_status));
        SerialUSB.print(")");
        SerialUSB.println();
        SerialUSB.println("Error stack: ");
        for(auto &st : status_stack) {
            if(st != PlatformStatus::STATUS_OK) {
                SerialUSB.print("\t0x");
                SerialUSB.print(statuscode(st), HEX);
                SerialUSB.print("(");
                SerialUSB.print(statusname(st));
                SerialUSB.print(")");
                SerialUSB.println();
            }
        }
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