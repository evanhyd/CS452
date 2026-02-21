#pragma once

#include "main_server.h"

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string_view>

namespace cmd {

struct Command {
  virtual void process(k4::ServerState& state) const;
};

namespace detail {

struct SetTrainSpeedCommand : Command {
  static constexpr std::string_view USAGE = "tr <train number> <train speed> - Set train speed";

  void process(k4::ServerState& state) const override;

  SetTrainSpeedCommand(unsigned trainNo, unsigned speed) : trainNo_{trainNo}, speed_{speed} {}

  unsigned trainNo_;
  unsigned speed_;
};

struct ReverseTrainCommand : Command {
  static constexpr std::string_view USAGE = "rv <train number> - Reverse train direction";

  void process(k4::ServerState& state) const override;

  ReverseTrainCommand(unsigned trainNo) : trainNo_{trainNo} {}

  unsigned trainNo_;
};

struct ThrowSwitchCommand : Command {
  static constexpr std::string_view USAGE =
      "sw <switch number> <switch direction> - Throw switch to straight (S) or curved (C)";

  void process(k4::ServerState& state) const override;

  ThrowSwitchCommand(unsigned switchNo, bool direction) : switchNo_{switchNo}, direction_{direction} {}

  unsigned switchNo_;
  bool direction_; // true = straight, false = curved
};

struct QuitCommand : Command {
  static constexpr std::string_view USAGE = "q - Quit program and reboot";

  void process(k4::ServerState& state) const override;
};

template <typename C>
concept CommandType = std::derived_from<C, struct Command> && requires {
  { C::USAGE } -> std::convertible_to<std::string_view>;
};

struct InvalidCommand : Command {
  void process(k4::ServerState& state) const override;

  InvalidCommand(std::string_view usage) : usage_{usage} {}

  std::string_view usage_;
};

template <CommandType... Cs> static constexpr auto max_size = std::max({sizeof(InvalidCommand), sizeof(Cs)...});

inline constexpr size_t MAX_COMMAND_SIZE =
    max_size<SetTrainSpeedCommand, ReverseTrainCommand, ThrowSwitchCommand, QuitCommand>;

} // namespace detail

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

  const Command* parse() {
    const Command* ret = parse_impl();
    clear();
    return ret;
  }

  operator std::string_view() const { return std::string_view{buffer_, length_}; }

  static constexpr bool isCmdChar(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == ' ';
  }

private:
  char* end() { return buffer_ + length_; }

  template <detail::CommandType C> detail::InvalidCommand* invalid() {
    return new (curr_cmd_) detail::InvalidCommand{C::USAGE};
  }

  const Command* parse_impl();

  char buffer_[MAX_COMMAND_LENGTH];
  char curr_cmd_[detail::MAX_COMMAND_SIZE];
  size_t length_;
};

constexpr uint16_t convSpeed(unsigned speed) { return static_cast<uint16_t>(speed > 0 ? 1 + (speed - 1) * 77 : 0); }

} // namespace cmd
