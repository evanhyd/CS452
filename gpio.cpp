#include "gpio.h"
#include <cstdint>

namespace gpio {

namespace {

constexpr uintptr_t GPIO_BASE = 0xFE000000 + 0x200000;

template <uintptr_t Offset> struct RegAccessor {
  static volatile uint32_t& operator()(uint32_t reg) {
    return *reinterpret_cast<volatile uint32_t*>(GPIO_BASE + Offset + reg * 4);
  }
};

constexpr RegAccessor<0x00> GPFSEL_REG;
constexpr RegAccessor<0xe4> GPIO_PUP_PDN_CNTRL_REG;
constexpr RegAccessor<0x40> GPEDS_REG;
constexpr RegAccessor<0x70> GPLEN_REG;

enum class Function : uint32_t {
  Input = 0x00,
  Output = 0x01,
  AltFn0 = 0x04,
  AltFn1 = 0x05,
  AltFn2 = 0x06,
  AltFn3 = 0x07,
  AltFn4 = 0x03,
  AltFn5 = 0x02
};

enum class Resistor : uint32_t { None = 0x00, PullUp = 0x01, PullDown = 0x02 };

void setup(uint32_t pin, Function setting, Resistor resistor) {
  uint32_t reg = pin / 10;
  uint32_t shift = (pin % 10) * 3;
  uint32_t status = GPFSEL_REG(reg);                   // read status
  status &= ~(7u << shift);                            // clear bits
  status |= (static_cast<uint32_t>(setting) << shift); // set bits
  GPFSEL_REG(reg) = status;

  reg = pin / 16;
  shift = (pin % 16) * 2;
  status = GPIO_PUP_PDN_CNTRL_REG(reg);                 // read status
  status &= ~(3u << shift);                             // clear bits
  status |= (static_cast<uint32_t>(resistor) << shift); // set bits
  GPIO_PUP_PDN_CNTRL_REG(reg) = status;                 // write back
}

void set_pin_low_detect(uint32_t pin, int enable) {
  uint32_t reg = pin / 32;
  uint32_t shift = pin % 32;
  if (enable)
    GPLEN_REG(reg) |= (1 << shift); // enable pin low detect
  else
    GPLEN_REG(reg) &= ~(1 << shift); // disable pin low detect
}

void init_interrupt() {
  setup(17, Function::Input, Resistor::None); // configure MCP2515_INT pin
  set_pin_low_detect(17, 1);                  // enable low detect on MCP2515_INT pin
}

} // namespace

// GPIO pins 14 & 15 already configured by boot loader, but redo for clarity.
void init() {
  setup(8, Function::AltFn0, Resistor::None);  // SPI0_CE0_N
  setup(9, Function::AltFn0, Resistor::None);  // SPI0_MISO
  setup(10, Function::AltFn0, Resistor::None); // SPI0_MOSI
  setup(11, Function::AltFn0, Resistor::None); // SPI0_SCLK

  setup(14, Function::AltFn0, Resistor::None); // UART TXD0
  setup(15, Function::AltFn0, Resistor::None); // UART

  init_interrupt();
}

uint32_t get_event_detect_status(uint32_t pin) {
  uint32_t reg = pin / 32;
  uint32_t shift = pin % 32;
  return (GPEDS_REG(reg) >> shift) & 0x01; // return the bit corresponding to the pin
}

void clr_event_detect_status(uint32_t pin) {
  uint32_t reg = pin / 32;
  uint32_t shift = pin % 32;
  GPEDS_REG(reg) = (1 << shift); // clear the event detect status for the pin
}

} // namespace gpio
