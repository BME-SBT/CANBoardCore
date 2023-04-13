
#include <Arduino.h>
#include <cstdarg>
#include <mbed_fault_handler.h>

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

extern "C" {

int WRAPPER_FUNC(putchar)(int c) {
    char cc = (char)c;
    SerialUSB.print(cc);
    return c;
}

int WRAPPER_FUNC(puts)(const char *s) {
    int len = (int)strlen(s);
    SerialUSB.println(s);
    return len;
}

int WRAPPER_FUNC(printf)(const char *fmt, ...) {
    va_list va;
    va_start(va, fmt);
    int ret = vprintf(fmt, va);
    va_end(va);
    return ret;
}
char vprintf_buffer[1024];
int WRAPPER_FUNC(vprintf)(const char *format, va_list va) {
    int r = vsnprintf(vprintf_buffer, 1024, format, va);
    SerialUSB.println(vprintf_buffer);
    return r;
}

static char error_out[2048];
static char error_fmtbuf[2048];

extern void mbed_error_printf(const char *fmt, ...);
void WRAPPER_FUNC(mbed_error_printf)(const char *fmt, ...) {
    va_list va;
    va_start(va, fmt);
    vsnprintf(error_fmtbuf, 2048,fmt,va);
    strcat(error_fmtbuf, error_out);
    va_end(va);
}
void WRAPPER_FUNC(print_context_info)(mbed_fault_context_t mbed_fault_context);
void WRAPPER_FUNC(mbed_fault_handler)(
    uint32_t fault_type, const mbed_fault_context_t *mbed_fault_context_in) {
    error_out[0] = 0;

    /* Need to set the flag to ensure prints do not trigger a "mutex in ISR"
     * trap if they're first prints since boot and we have to init the I/O
     * system.
     */
    mbed_error_printf("\n++ MbedOS Fault Handler ++\n\nFaultType: ");

    switch (fault_type) {
    case MEMMANAGE_FAULT_EXCEPTION:
        mbed_error_printf("MemManageFault");
        break;

    case BUS_FAULT_EXCEPTION:
        mbed_error_printf("BusFault");
        break;

    case USAGE_FAULT_EXCEPTION:
        mbed_error_printf("UsageFault");
        break;

    // There is no way we can hit this code without getting an exception, so we
    // have the default treated like hardfault
    case HARD_FAULT_EXCEPTION:
    default:
        mbed_error_printf("HardFault");
        break;
    }
    mbed_error_printf("\n\nContext:");
    __wrap_print_context_info(*mbed_fault_context_in);

    mbed_error_printf("\n\n-- MbedOS Fault Handler --\n\n");
    SerialUSB.println(error_out);
    while(1);
}
void WRAPPER_FUNC(print_context_info)(mbed_fault_context_t mbed_fault_context)
{
    //Context Regs
    for (int i = 0; i < 13; i++) {
        mbed_error_printf("\nR%-4d: %08" PRIX32, i, (&mbed_fault_context.R0_reg)[i]);
    }

    mbed_error_printf("\nSP   : %08" PRIX32
                      "\nLR   : %08" PRIX32
                      "\nPC   : %08" PRIX32
                      "\nxPSR : %08" PRIX32
                      "\nPSP  : %08" PRIX32
                      "\nMSP  : %08" PRIX32, mbed_fault_context.SP_reg, mbed_fault_context.LR_reg, mbed_fault_context.PC_reg,
                      mbed_fault_context.xPSR, mbed_fault_context.PSP, mbed_fault_context.MSP);

    //Capture CPUID to get core/cpu info
    mbed_error_printf("\nCPUID: %08" PRIX32, SCB->CPUID);

#if !defined(TARGET_M0) && !defined(TARGET_M0P) && !defined(TARGET_M23)
    //Capture fault information registers to infer the cause of exception
    mbed_error_printf("\nHFSR : %08" PRIX32
                      "\nMMFSR: %08" PRIX32
                      "\nBFSR : %08" PRIX32
                      "\nUFSR : %08" PRIX32
                      "\nDFSR : %08" PRIX32
                      "\nAFSR : %08" PRIX32  ////Split/Capture CFSR into MMFSR, BFSR, UFSR
                      , SCB->HFSR, (0xFF & SCB->CFSR), ((0xFF00 & SCB->CFSR) >> 8), ((0xFFFF0000 & SCB->CFSR) >> 16), SCB->DFSR, SCB->AFSR);

    //Print MMFAR only if its valid as indicated by MMFSR
    if ((0xFF & SCB->CFSR) & 0x80) {
        mbed_error_printf("\nMMFAR: %08" PRIX32, SCB->MMFAR);
    }
    //Print BFAR only if its valid as indicated by BFSR
    if (((0xFF00 & SCB->CFSR) >> 8) & 0x80) {
        mbed_error_printf("\nBFAR : %08" PRIX32, SCB->BFAR);
    }
#endif

    //Print Mode
    if (mbed_fault_context.EXC_RETURN & 0x8) {
        mbed_error_printf("\nMode : Thread");
        //Print Priv level in Thread mode - We capture CONTROL reg which reflects the privilege.
        //Note that the CONTROL register captured still reflects the privilege status of the
        //thread mode eventhough we are in Handler mode by the time we capture it.
        if (mbed_fault_context.CONTROL & 0x1) {
            mbed_error_printf("\nPriv : User");
        } else {
            mbed_error_printf("\nPriv : Privileged");
        }
    } else {
        mbed_error_printf("\nMode : Handler");
        mbed_error_printf("\nPriv : Privileged");
    }
    //Print Return Stack
    if (mbed_fault_context.EXC_RETURN & 0x4) {
        mbed_error_printf("\nStack: PSP");
    } else {
        mbed_error_printf("\nStack: MSP");
    }
}

}
/**
 * Global setup code. Runs before user setup.
 */
void global_setup() {
    // Global setup
    bool err = true;
    while (err) {
        LOG("initializing CAN");
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

    SerialUSB.print("\ttx_req_byte_count: ");
    SerialUSB.println(g_can_stat.tx_req_byte_count);

    SerialUSB.print("\tapprox_required_tx_speed: ");
    SerialUSB.print((double) g_can_stat.tx_req_byte_count / (millis() / 1000.0));
    SerialUSB.println(" bps");

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