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
    u64 loop_count;
    u64 app_loop_time;
    u64 postloop_time;
    u64 user_loop_time;
    u32 irq_per_loop;
    u32 yield_count;
} g_stats;
static AppState g_state = AppState::SETUP;

/**
 * Global setup code. Runs before user setup.
 */
void global_setup() {
    // Global setup
    bool err = true;
    while (err) {
        log("initializing CAN");
        err = PLATFORM_CAN.init();
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

void print_can_stats() {

    static u64 last_tx_count;
    static u64 last_rx_count;

    SerialUSB.println("CAN stats: ");

    SerialUSB.print("\ttx_byte_count: ");
    SerialUSB.println(g_can_stat.tx_byte_count);

    SerialUSB.print("\trx_byte_count: ");
    SerialUSB.println(g_can_stat.rx_byte_count);

    SerialUSB.print("\ttx_dropped_frame_count: ");
    SerialUSB.println(g_can_stat.tx_dropped_frame_count);

    SerialUSB.print("\ttx_sent_frame_count: ");
    SerialUSB.println(g_can_stat.tx_sent_frame_count);

    SerialUSB.print("\ttx_queued_frame_count: ");
    SerialUSB.println(g_can_stat.tx_queued_frame_count);

    SerialUSB.print("\trx_dropped_frame_count: ");
    SerialUSB.println(g_can_stat.rx_dropped_frame_count);

    SerialUSB.print("\trx_received_frame_count: ");
    SerialUSB.println(g_can_stat.rx_received_frame_count);

    SerialUSB.print("\trx_processed_frame_count: ");
    SerialUSB.println(g_can_stat.rx_processed_frame_count);

    SerialUSB.print("\tisr_lost_race_count: ");
    SerialUSB.println(g_can_stat.isr_lost_race_count);

    SerialUSB.print("\tapprox_tx_speed: ");
    SerialUSB.print(g_can_stat.tx_byte_count - last_tx_count);
    last_tx_count = g_can_stat.tx_byte_count;
    SerialUSB.println(" bps");

    SerialUSB.print("\tapprox_rx_speed: ");
    SerialUSB.print(g_can_stat.rx_byte_count - last_rx_count);
    last_rx_count = g_can_stat.rx_byte_count;
    SerialUSB.println(" bps");
}

void print_loop_stats() {
    SerialUSB.println("Loop stats: ");
    SerialUSB.print("\tlast_loop_time: ");
    SerialUSB.print(g_stats.last_loop_time);
    SerialUSB.println(" us");

    double avg_lt = (double)micros() / (double)g_stats.loop_count;
    SerialUSB.print("\tavg_loop_time: ");
    SerialUSB.print(avg_lt);
    SerialUSB.println(" us");

    SerialUSB.print("\tapp_loop_time: ");
    SerialUSB.print(g_stats.app_loop_time);
    SerialUSB.println(" us");

    SerialUSB.print("\tuser_loop_time: ");
    SerialUSB.print(g_stats.user_loop_time);
    SerialUSB.println(" us");

    SerialUSB.print("\tpostloop_time: ");
    SerialUSB.print(g_stats.postloop_time);
    SerialUSB.println(" us");

    SerialUSB.print("\tirq_per_loop: ");
    SerialUSB.print(g_stats.irq_per_loop);
    SerialUSB.println("");

    SerialUSB.print("\tyield_count: ");
    SerialUSB.print(g_stats.yield_count);
    SerialUSB.println("");
}

/**
 * Main loop code. Calls the app's loop, checks errors
 */
void user_loop() {

    u64 app_loop_start = micros();
    PlatformStatus err = app_loop();
    g_stats.app_loop_time = micros() - app_loop_start;
    if (is_err(err)) {
        platform_set_status(err);
        g_state = AppState::FATAL;
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
        for (auto &st : status_stack) {
            if (st != PlatformStatus::STATUS_OK) {
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
    u64 user_loop_time = micros() - loop_start;
    platform_postloop();
    u64 postloop_time = (micros() - loop_start) - user_loop_time;

    // Calculate loop time
    u64 loop_time = micros() - loop_start;
    // TODO: Validate loop time
    g_stats.last_loop_time = loop_time;
    g_stats.user_loop_time = user_loop_time;
    g_stats.postloop_time = postloop_time;
    g_stats.loop_count++;
    static u32 last_irq_count;

    g_stats.irq_per_loop = g_can_stat.irq_handled - last_irq_count;
    last_irq_count = g_can_stat.irq_handled;

    schedule_call(print_loop_stats, 1000);
    schedule_call(print_can_stats, 1000);
}