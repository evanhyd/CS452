#pragma once
#include <cstdint>

namespace mcp2515 {

enum class Control : uint8_t {
  Go = 0x01,
  Halt = 0x02,
  Stop = 0x00,
};

enum class Direction : uint8_t {
  Forward = 1,
  Backward = 2,
  Reverse = 3,
};

enum class SwitchState : uint8_t {
  Curved = 0,
  Straight = 1,
};

struct SensorEvent {
  char bank;
  uint8_t number;
  bool old_occupied;
  bool new_occupied;
};

struct MMessage {
  uint32_t uid;
  uint8_t dlc;
  uint8_t data[8];

  static MMessage control(Control c);
  static MMessage set_speed(uint32_t train_no, uint8_t step);
  static MMessage set_direction(uint32_t train_no, Direction dir);
  static MMessage set_light(uint32_t train_no, bool on);
  static MMessage set_switch(uint32_t switch_no, SwitchState state);
};

enum class CanInterruptType : uint8_t {
  RX0IE = 1 << 0,
  RX1IE = 1 << 1,
  TX0IE = 1 << 2,
  TX1IE = 1 << 3,
  TX2IE = 1 << 4,
  ERRIE = 1 << 5,
  WAKIE = 1 << 6,
  MERRE = 1 << 7
};

// Initialize the MCP2515 CAN controller. Should be called after initializing GPIO and SPI.
void init();

// Enable the can interrupt.
void setInterruptEnabled(CanInterruptType interruptType, bool isEnabled);

// Get the OR masks of all the on-going interrupt.
uint8_t getInterruptStatus();

// Acknowledge the interrupt and clears it.
void clearActiveInterrupt(CanInterruptType interruptType);

} // namespace mcp2515
