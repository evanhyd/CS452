#pragma once

#include "marklin/marklin_train_track.h"
#include <cstddef>
#include <cstdint>

namespace cmd {

enum class CmdTag : uint8_t {
  None,
  Invalid,
  SetSpeed,
  Reverse,
  SetSwitch,
  Quit,
};

struct ParsedCommand {
  CmdTag tag;
  struct Empty {};
  struct SetSpeedData {
    unsigned trainId;
    unsigned speed;
  };
  struct ReverseData {
    unsigned trainId;
  };
  struct SetSwitchData {
    unsigned switchId;
    marklin::SwitchState state;
  };
  struct InvalidData {
    const char* usage;
  };
  union {
    Empty empty;
    SetSpeedData setSpeed;
    ReverseData reverse;
    SetSwitchData setSwitch;
    InvalidData invalid;
  };
};

// A command buffer that holds all the user input.
// When the user enters and submits the command, it parses to the corresponding ParsedCommand type.
class CommandBuffer {
public:
  static constexpr size_t MAX_COMMAND_LENGTH = 128;

  CommandBuffer() : length_{0} {}

  void push(char c) {
    if (length_ < MAX_COMMAND_LENGTH) {
      buffer_[length_++] = c;
    }
  }

  void backspace() {
    if (length_ > 0) {
      --length_;
    }
  }

  void clear() { length_ = 0; }

  ParsedCommand parse() {
    ParsedCommand ret = parse_impl();
    clear();
    return ret;
  }

  size_t length() const { return length_; }
  const char* data() const { return buffer_; }

  static constexpr bool isCmdChar(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == ' ';
  }

private:
  const char* end() const { return buffer_ + length_; }

  ParsedCommand parse_impl();

  char buffer_[MAX_COMMAND_LENGTH];
  size_t length_;
};

constexpr uint16_t convSpeed(unsigned speed) { return static_cast<uint16_t>(speed > 0 ? 1 + (speed - 1) * 77 : 0); }

} // namespace cmd
