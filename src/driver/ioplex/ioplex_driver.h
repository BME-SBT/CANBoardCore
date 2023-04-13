#pragma once

#include "ioplex.pio.h"
#include <pico/types.h>
#include <cstdint>
#include <cstdio>

/*       1
 *    ------
 *   |      \
 *128|   4  \  2
 *    ------
 *   |      \  16
 *64 |  32  \
 *    ------   . 8
 */

#define TOP 0x01
#define TR 0x02
#define TL 0x80
#define MID 0x04
#define BR 0x10
#define BL 0x40
#define BOT 0x20
#define DOT 0x08

// Digits
const uint8_t digits[16] = {
    (TOP | TL | TR | BR | BL | BOT),       // 0
    (TR | BR),                             // 1
    (TOP | TR | MID | BL | BOT),           // 2
    (TOP | TR | MID | BR | BOT),           // 3
    (TR | TL | MID | BR),                  // 4
    (TOP | TL | MID | BR | BOT),           // 5
    (TOP | TL | MID | BR | BL | BOT),      // 6
    (TOP | TR | BR),                       // 7
    (TOP | TL | TR | MID | BR | BL | BOT), // 8
    (TOP | TR | TL | MID | BR | BOT),      // 9
    (TOP | TR | TL | MID | BR | BL),       // A
    (TL | BL | MID | BR | BOT),            // b
    (MID | BL | BOT),                      // c
    (TR | MID | BR | BL | BOT),            // d
    (TOP | TL | MID | BL | BOT),           // E
    (TOP | TL | MID | BL),                 // F
};

// LEDs
#define LED_1 0x100
#define LED_2 0x80000000

// Buttons
#define BTN 0x80000000
#define DIP 0x7F800000
#define INPUT_SHIFT 23

struct SwitchValue {
    bool buttonPressed;
    uint8_t dipValue;
};

class IOPlexDriver {

public:
  IOPlexDriver(PIO pio, uint stateMachine, uint8_t pinSegBase, uint8_t pinDigBase): m_pio(pio), m_stateMachine(stateMachine) {
    m_offset = pio_add_program(m_pio, &ioplex_program);
    ioplex_init(m_pio, m_stateMachine, m_offset, pinSegBase, pinDigBase);
  }

  void setLED1(bool enabled) { m_led1 = enabled; send_update();}
  void setLED2(bool enabled) { m_led2 = enabled; send_update();}
  void setDigit1(uint8_t value) { m_digit1 = value; send_update();}
  void setDigit2(uint8_t value) { m_digit2 = value; send_update(); }
  void setDigitNum(uint8_t value) {
    setDigit2(value & 0x0F);
    setDigit1((value & 0xF0) >> 4);
    send_update();
  }

  SwitchValue getSwitchState(bool blocking = false) {
      uint32_t value;
      if(blocking) {
          value = pio_sm_get_blocking(m_pio, m_stateMachine);
      }else {
          value = pio_sm_get(m_pio, m_stateMachine);
      }
      
      uint8_t button_val = (value & BTN) >> INPUT_SHIFT;
      uint8_t dip_val = (value & DIP) >> INPUT_SHIFT;
      return {.buttonPressed = button_val ? true : false, .dipValue = dip_val};
  }



private:
  void send_update() {
    uint32_t state = 0;

    // Map higher digit
    state |= m_digit1 < 16 ? digits[m_digit1] : 0;

    // Map lower digit
    state |= (m_digit2 < 16 ? digits[m_digit2] : 0) << 9;

    // Led 1
    state |= m_led1 ? LED_1 : 0;

    // Led 2
    state |= m_led2 ? LED_2 : 0;

    pio_sm_put(m_pio, m_stateMachine, state);
  }

private:
  PIO m_pio;
  uint m_stateMachine;
  uint m_offset;

  // State
  bool m_led1 = false;
  bool m_led2 = false;

  uint8_t m_digit1 = 0;
  uint8_t m_digit2 = 0;
};