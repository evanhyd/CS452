#pragma once
#include "marklin/marklin_message.h"
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

enum ReadStatusMask : uint8_t {
  RX0IF = 1 << 0,  // Receive buffer 0 contains a message
  RX1IF = 1 << 1,  // Receive buffer 1 contains a message
  TX0REQ = 1 << 2, // Transmit buffer 0 is pending transmission
  TX0IF = 1 << 3,  // Transmit buffer 0 is empty (interrupt flag)
  TX1REQ = 1 << 4, // Transmit buffer 1 is pending transmission
  TX1IF = 1 << 5,  // Transmit buffer 1 is empty (interrupt flag)
  TX2REQ = 1 << 6, // Transmit buffer 2 is pending transmission
  TX2IF = 1 << 7,  // Transmit buffer 2 is empty (interrupt flag)
};

// CANINTF and CANINTE masks.
enum CanInterruptMask : uint8_t {
  RX0IE = 1 << 0, // Receive Buffer 0 Full Interrupt Flag bit
  RX1IE = 1 << 1, // Receive Buffer 1 Full Interrupt Flag bit
  TX0IE = 1 << 2, // Transmit Buffer 0 Empty Interrupt Flag bit
  TX1IE = 1 << 3, // Transmit Buffer 1 Empty Interrupt Flag bit
  TX2IE = 1 << 4, // Transmit Buffer 2 Empty Interrupt Flag bit
  ERRIE = 1 << 5, // Error Interrupt Flag bit (multiple sources in EFLG register)
  WAKIE = 1 << 6, // Wake-up Interrupt Flag bit
  MERRE = 1 << 7, // Message Error Interrupt Flag bit
  ReceiveAndTransmit = RX0IE | RX1IE | TX0IE,
};

// Initialize the MCP2515 CAN controller. Should be called after initializing GPIO and SPI.
void init();

// Enable the can interrupt.
void setInterruptEnabled(CanInterruptMask interruptType, bool isEnabled);

// Get the OR masks of all the on-going interrupt.
uint8_t getInterruptFlags();

// Acknowledge the interrupt and clears it.
void clearInterrupt(CanInterruptMask interruptType);

// Send a marklin message to the CAN bus via TX0 buffer.
// Precondition: The transmit buffer is ready.
void sendMessage(const marklin::MMessage& message);

// Read a marklin message from the buffer.
// Precondition: The receive buffer is non empty.
marklin::MMessage receiveMessage(RxBuffer buffer);

} // namespace mcp2515
