// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include <hardware/pio.h>
#endif

// ------ //
// ioplex //
// ------ //

#define ioplex_wrap_target 0
#define ioplex_wrap 23

static const uint16_t ioplex_program_instructions[] = {
            //     .wrap_target
    0x8080, //  0: pull   noblock                    
    0xa027, //  1: mov    x, osr                     
    0xe081, //  2: set    pindirs, 1                 
    0xe000, //  3: set    pins, 0                    
    0x7f09, //  4: out    pins, 9                [31]
    0xbf42, //  5: nop                           [31]
    0xbf42, //  6: nop                           [31]
    0xbf42, //  7: nop                           [31]
    0xbf42, //  8: nop                           [31]
    0xbf42, //  9: nop                           [31]
    0xbf42, // 10: nop                           [31]
    0xe001, // 11: set    pins, 1                    
    0x7f09, // 12: out    pins, 9                [31]
    0xbf42, // 13: nop                           [31]
    0xbf42, // 14: nop                           [31]
    0xbf42, // 15: nop                           [31]
    0xbf42, // 16: nop                           [31]
    0xbf42, // 17: nop                           [31]
    0xbf42, // 18: nop                           [31]
    0x6097, // 19: out    pindirs, 23                
    0x4009, // 20: in     pins, 9                    
    0x8000, // 21: push   noblock                    
    0xa0e2, // 22: mov    osr, y                     
    0x6089, // 23: out    pindirs, 9                 
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program ioplex_program = {
    .instructions = ioplex_program_instructions,
    .length = 24,
    .origin = -1,
};

static inline pio_sm_config ioplex_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + ioplex_wrap_target, offset + ioplex_wrap);
    return c;
}

#ifdef __cplusplus
extern "C" {
#endif
#include <hardware/clocks.h>
#include <hardware/gpio.h>
static inline void ioplex_init(PIO pio, uint sm, uint offset, uint seg_pin, uint dig_pin)
{
    pio_sm_config c = ioplex_program_get_default_config(offset);
    sm_config_set_out_pins(&c, seg_pin, 9);
    sm_config_set_set_pins(&c, dig_pin, 1);
    sm_config_set_in_pins(&c, seg_pin);
    float div = (float)(clock_get_hz(clk_sys) / 457000);
    sm_config_set_clkdiv(&c, div);
    // sm_config_set_clkdiv_int_frac(&c, 53200, 0);
    for (int i = 0; i < 9; i++)
    {
        gpio_pull_down(seg_pin + i);
    }
    for (int i = 0; i < 8; i++)
    {
        pio_sm_exec(pio, sm, pio_encode_set(pio_y, 0xf));
        pio_sm_exec(pio, sm, pio_encode_in(pio_y, 4));
    }
    pio_sm_exec(pio, sm, pio_encode_mov(pio_y, pio_isr));
    for (int i = 0; i < 9; i++)
    {
        pio_gpio_init(pio, seg_pin + i);
    }
    pio_gpio_init(pio, dig_pin);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}
#ifdef __cplusplus
};
#endif

#endif

