#include "spi.h"
#include <cstdint>

namespace spi {

namespace {

constexpr uintptr_t SPI0_BASE = 0xFE000000 + 0x204000;

volatile uint32_t& spi0_reg(uintptr_t offset) { return *reinterpret_cast<volatile uint32_t*>(SPI0_BASE + offset); }

// SPI register offsets
constexpr uintptr_t CS = 0x00;
constexpr uintptr_t FIFO = 0x04;
constexpr uintptr_t CLK = 0x08;

// CS bit fields
constexpr uint32_t CS_TXD = 0x00040000;
constexpr uint32_t CS_RXD = 0x00020000;
constexpr uint32_t CS_DONE = 0x00010000;
constexpr uint32_t CS_TA = 0x00000080;
constexpr uint32_t CS_CLEAR_RX = 0x00000020;
constexpr uint32_t CS_CLEAR_TX = 0x00000010;
constexpr uint32_t CS_CPOL = 0x00000008;
constexpr uint32_t CS_CPHA = 0x00000004;
constexpr uint32_t CS_CS_10 = 0x00000002;
constexpr uint32_t CS_CS_01 = 0x00000001;

} // namespace

void init() {
  uint32_t ctrl = spi0_reg(CS);
  ctrl &= ~(CS_CS_01 | CS_CS_10);    // CS = 0 (SPI0_CE0_N)
  ctrl &= ~CS_CPOL;                  // CPOL = 0
  ctrl &= ~CS_CPHA;                  // CPHA = 0
  ctrl |= CS_CLEAR_RX | CS_CLEAR_TX; // Clear RX and TX FIFOs
  spi0_reg(CS) = ctrl;

  // Core clock = 500 MHz (see core_freq in
  // https://www.raspberrypi.com/documentation/computers/config_txt.html#overclocking)
  uint32_t cdiv = 500 / 10; // Target clock is 10 MHz (MCP2515 data sheet, Page 1)
  cdiv += cdiv % 2;         // Round up to nearest even number
  spi0_reg(CLK) = cdiv;     // SCLK = Core Clock / CDIV
}

void begin_transaction() {
  uint32_t ctrl = spi0_reg(CS);
  ctrl |= CS_TA; // Transfer active
  spi0_reg(CS) = ctrl;
}

uint8_t transfer_one(uint8_t tx_byte) {
  while (!(spi0_reg(CS) & CS_TXD))
    ; // Wait for space in TX FIFO

  spi0_reg(FIFO) = tx_byte; // Write byte to TX FIFO
  while (!(spi0_reg(CS) & CS_RXD))
    ; // Wait for data in RX FIFO

  return static_cast<uint8_t>(spi0_reg(FIFO)); // Read byte from RX FIFO
}

void end_transaction() {
  while (!(spi0_reg(CS) & CS_DONE))
    ; // Wait for transfer to complete
  uint32_t ctrl = spi0_reg(CS);
  ctrl &= ~CS_TA; // Transfer inactive
  spi0_reg(CS) = ctrl;
}

} // namespace spi
