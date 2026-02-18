#include "spi.h"
#include <cstddef>

namespace {
struct SpiInstance {
  volatile uint32_t CS;   // 0x00: SPI Master Control and Status
  volatile uint32_t FIFO; // 0x04: SPI Master TX and RX FIFOs
  volatile uint32_t CLK;  // 0x08: SPI Master Clock Divider
  volatile uint32_t DLEN; // 0x0C: SPI Master Data Length
  volatile uint32_t LTOH; // 0x10: SPI LoSSI mode TOH
  volatile uint32_t DC;   // 0x14: SPI DMA DREQ Controls
};

volatile SpiInstance* instance = (SpiInstance*)(0xFE000000 + 0x204000);

// CS bit fields
constexpr uint32_t SPI_CS_TXD = 0x00040000;
constexpr uint32_t SPI_CS_RXD = 0x00020000;
constexpr uint32_t SPI_CS_DONE = 0x00010000;
constexpr uint32_t SPI_CS_TA = 0x00000080;
constexpr uint32_t SPI_CS_CLEAR_RX = 0x00000020;
constexpr uint32_t SPI_CS_CLEAR_TX = 0x00000010;
constexpr uint32_t SPI_CS_CPOL = 0x00000008;
constexpr uint32_t SPI_CS_CPHA = 0x00000004;
constexpr uint32_t SPI_CS_CS_10 = 0x00000002;
constexpr uint32_t SPI_CS_CS_01 = 0x00000001;
} // namespace

namespace spi {
void init() {
  uint32_t ctrl = instance->CS;
  ctrl &= ~(SPI_CS_CS_01 | SPI_CS_CS_10);    // CS = 0 (SPI0_CE0_N)
  ctrl &= ~SPI_CS_CPOL;                      // CPOL = 0
  ctrl &= ~SPI_CS_CPHA;                      // CPHA = 0
  ctrl |= SPI_CS_CLEAR_RX | SPI_CS_CLEAR_TX; // Clear RX and TX FIFOs
  instance->CS = ctrl;

  // Core clock = 500 MHz (see core_freq in
  // https://www.raspberrypi.com/documentation/computers/config_txt.html#overclocking)
  uint32_t cdiv = 500 / 10; // Target clock is 10 MHz (MCP2515 data sheet, Page 1)
  cdiv += cdiv % 2;         // Round up to nearest even number
  instance->CLK = cdiv;     // SCLK = Core Clock / CDIV
}

void beginTransaction() {
  instance->CS |= SPI_CS_TA; // Transfer active
}

void endTransaction() {
  while (!(instance->CS & SPI_CS_DONE))
    ;                         // Wait for transfer to complete
  instance->CS &= ~SPI_CS_TA; // Transfer inactive
}

uint8_t transferOne(uint8_t tx_byte) {
  while (!(instance->CS & SPI_CS_TXD))
    ;                       // Wait for space in TX FIFO
  instance->FIFO = tx_byte; // Write byte to TX FIFO
  while (!(instance->CS & SPI_CS_RXD))
    ;                             // Wait for data in RX FIFO
  return uint8_t(instance->FIFO); // Read byte from RX FIFO
}
} // namespace spi
