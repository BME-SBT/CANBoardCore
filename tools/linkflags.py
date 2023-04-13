Import("env")

#
# Dump build environment (for debug)
# print(env.Dump())
#

env.Append(
    LINKFLAGS=[
        "-Wl,--wrap=printf",
        "-Wl,--wrap=vprintf",
        "-Wl,--wrap=puts",
        "-Wl,--wrap=putchar",
        "-Wl,--wrap=mbed_fault_handler"
        "-Wl,--wrap=mbed_error_printf"
        "-Wl,--wrap=print_context_info"
    ]
)