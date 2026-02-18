#pragma once
#include "marklin_message.h"
#include <cstdint>

/**
MCP 2515 controller
    A wrapper around the CAN bus protocol, responsible for translating the signals.
 */
namespace mcp2515 {

enum class RxBuffer : uint8_t {
  Rx0 = 0,
  Rx1 = 1,
};

enum class CanInterruptType : uint8_t {
  RX0IE = 1 << 0,
  RX1IE = 1 << 1,
  TX0IE = 1 << 2,
  TX1IE = 1 << 3,
  TX2IE = 1 << 4,
  ERRIE = 1 << 5,
  WAKIE = 1 << 6,
  MERRE = 1 << 7,
  ReceiveAndTransmit = RX0IE | RX1IE | TX0IE,
};

// Initialize the MCP2515 CAN controller. Should be called after initializing GPIO and SPI.
void init();

// Enable the can interrupt.
void setInterruptEnabled(CanInterruptType interruptType, bool isEnabled);

// Get the OR masks of all the on-going interrupt.
uint8_t getInterruptStatus();

// Acknowledge the interrupt and clears it.
void clearActiveInterrupt(CanInterruptType interruptType);

// Send a marklin message to the CAN bus via TX0 buffer.
// Precondition: The transmit buffer is ready.
void sendMessage(const marklin::MMessage& message);

// Read a marklin message from the buffer.
// Precondition: The receive buffer is non empty.
marklin::MMessage receiveMessage(RxBuffer buffer);

} // namespace mcp2515
