#include "uart.h"
#include <cstdarg>
#include <cstdint>

namespace {
struct UartInstance {
  volatile uint32_t DR;            // 0x00: Data Register
  volatile uint32_t RSRECR;        // 0x04: Receive Status Register/Error Clear Register
  volatile uint32_t reserved0[4];  // 0x08 - 0x14: Reserved
  volatile uint32_t FR;            // 0x18: Flag Register
  volatile uint32_t reserved1;     // 0x1C: Reserved
  volatile uint32_t ILPR;          // 0x20: IrDA Low-Power Counter Register (Not in use)
  volatile uint32_t IBRD;          // 0x24: Integer Baud Rate Divisor
  volatile uint32_t FBRD;          // 0x28: Fractional Baud Rate Divisor
  volatile uint32_t LCRH;          // 0x2C: Line Control Register
  volatile uint32_t CR;            // 0x30: Control Register
  volatile uint32_t IFLS;          // 0x34: Interrupt FIFO Level Select Register
  volatile uint32_t IMSC;          // 0x38: Interrupt Mask Set Clear Register
  volatile uint32_t RIS;           // 0x3C: Raw Interrupt Status Register
  volatile uint32_t MIS;           // 0x40: Masked Interrupt Status Register
  volatile uint32_t ICR;           // 0x44: Interrupt Clear Register
  volatile uint32_t DMACR;         // 0x48: DMA Control Register
  volatile uint32_t reserved2[13]; // 0x4C - 0x7C: Reserved
  volatile uint32_t ITCR;          // 0x80: Test Control Register
  volatile uint32_t ITIP;          // 0x84: Integration Test Input Register
  volatile uint32_t ITOP;          // 0x88: Integration Test Output Register
  volatile uint32_t TDR;           // 0x8C: Test Data Register
  volatile uint32_t reserved3[93]; // 0x90 - 0x200: Reserved
};

// masks for specific fields in the UART registers
constexpr uint32_t UART_FR_BUSY = 0x08;
constexpr uint32_t UART_FR_RXFE = 0x10;
constexpr uint32_t UART_FR_TXFF = 0x20;
constexpr uint32_t UART_FR_RXFF = 0x40;
constexpr uint32_t UART_FR_TXFE = 0x80;

constexpr uint32_t UART_CR_UARTEN = 0x01;
constexpr uint32_t UART_CR_LBE = 0x80;
constexpr uint32_t UART_CR_TXE = 0x100;
constexpr uint32_t UART_CR_RXE = 0x200;
constexpr uint32_t UART_CR_RTS = 0x800;
constexpr uint32_t UART_CR_RTSEN = 0x4000;
constexpr uint32_t UART_CR_CTSEN = 0x8000;

constexpr uint32_t UART_LCRH_PEN = 0x02;
constexpr uint32_t UART_LCRH_EPS = 0x04;
constexpr uint32_t UART_LCRH_STP2 = 0x08;
constexpr uint32_t UART_LCRH_FEN = 0x10;
constexpr uint32_t UART_LCRH_WLEN_LOW = 0x20;
constexpr uint32_t UART_LCRH_WLEN_HIGH = 0x40;

constexpr uintptr_t MMIO_BASE = 0xFE000000;
volatile UartInstance* uartInstances = reinterpret_cast<volatile UartInstance*>(MMIO_BASE + 0x201000);
} // namespace

// Configure the line properties (e.g, parity, baud rate) of a UART and ensure that it is enabled
void Uart::configAndEnable(size_t line) {
  uint32_t baud_ival, baud_fval;
  uint32_t flag = UART_LCRH_FEN;

  switch (line) {
  // setting baudrate to approx. 115246.09844 (best we can do); 1 stop bit
  case CONSOLE: {
    baud_ival = 26;
    baud_fval = 2;
    break;
  }
  default:
    return;
  }

  // line control registers should not be changed while the UART is enabled, so disable it
  volatile UartInstance* uartInstance = uartInstances + line;
  uint32_t cr_state = uartInstance->CR;

  uartInstance->CR = cr_state & ~UART_CR_UARTEN;

  // set the baud rate
  uartInstance->IBRD = baud_ival;
  uartInstance->FBRD = baud_fval;

  // set the line control registers: 8 bit, no parity, 1 or 2 stop bits, FIFOs enabled
  uartInstance->LCRH = UART_LCRH_WLEN_HIGH | UART_LCRH_WLEN_LOW | flag;

  // re-enable the UART; enable both transmit and receive regardless of previous state
  uartInstance->CR = cr_state | UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE;
}

void Uart::syncPrint(size_t line, const char* cstring) {
  for (; *cstring; ++cstring) {
    while (uartInstances[line].FR & UART_FR_TXFF) {
    }
    uartInstances[line].DR = *cstring;
  }
}

char Uart::syncRead(size_t line) {
  while (uartInstances[line].FR & UART_FR_RXFE) {
  }
  return static_cast<char>(uartInstances[line].DR);
}
