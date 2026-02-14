#pragma once

#include <cstdint>

namespace mcp2515 {

struct MMessage {
  uint32_t uid;
  uint8_t dlc;
  uint8_t data[8];
};

/** Initialize the MCP2515 CAN controller. Should be called after initializing GPIO and SPI. */
void init();

void poll();

namespace cmd {

enum class Control : uint8_t {
  Go = 0x01,
  Halt = 0x02,
  Stop = 0x00,
};

void control(Control c);

void set_speed(uint32_t train_no, uint8_t step);

enum class Direction : uint8_t {
  Forward = 1,
  Backward = 2,
  Reverse = 3,
};

void set_direction(uint32_t train_no, Direction dir);

void set_light(uint32_t train_no, bool on);

enum class SwitchState : uint8_t {
  Curved = 0,
  Straight = 1,
};

void set_switch(uint32_t switch_no, SwitchState state);

struct SensorEvent {
  char bank;
  uint8_t number;
  bool old_occupied;
  bool new_occupied;
};

} // namespace cmd

} // namespace mcp2515
