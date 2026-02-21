#include "command.h"

#include "helper_tasks.h"
#include "main_server.h"

#include "kernel/syscalls.h"
#include "marklin/marklin_message.h"
#include "server_tasks/can_server.h"

namespace cmd {

void Command::process(k4::ServerState& state) const { state.status.set("Invalid command."); }

namespace detail {

void detail::SetTrainSpeedCommand::process(k4::ServerState& state) const {
  if (speed_ > 14) {
    state.status.set("Invalid speed %u. Must be between 0 and 14.", speed_);
    return;
  }
  if (trainNo_ == 0 || trainNo_ > k4::MAX_TRAINS) {
    state.status.set("Invalid train number %u. Must be between 1 and %u.", trainNo_, k4::MAX_TRAINS);
    return;
  }
  uint8_t trainNo = static_cast<uint8_t>(trainNo_);
  state.status.set("Set train %u to speed %u.", trainNo, speed_);
  state.initTrain(trainNo);
  state.trains[trainNo].speed = static_cast<uint8_t>(speed_);
  state.sendCAN(marklin::MMessage::setTrainSpeed(trainNo, convSpeed(speed_)));
}

void detail::ReverseTrainCommand::process(k4::ServerState& state) const {
  if (trainNo_ == 0 || trainNo_ > k4::MAX_TRAINS) {
    state.status.set("Invalid train number %u. Must be between 1 and %u.", trainNo_, k4::MAX_TRAINS);
    return;
  }
  uint8_t trainNo = static_cast<uint8_t>(trainNo_);
  // TODO: handle reverse while another reverse is in progress
  state.status.set("Reversing train %u.", trainNo);
  state.sendCAN(marklin::MMessage::setTrainSpeed(trainNo, 0));
  state.toReverse.push(trainNo);
  ::Create(4, k4::reverseWorkerTask);
}

void detail::ThrowSwitchCommand::process(k4::ServerState& state) const {
  state.status.set("Thrown switch %u to %c.", switchNo_, direction_ ? 'S' : 'C');
  // track::switch_manager.set(switch_no,
  //                           direction ? track::SwitchManager::State::Straight : track::SwitchManager::State::Curved);
}

extern "C" [[noreturn]] void _reboot();

void detail::QuitCommand::process(k4::ServerState&) const { _reboot(); }

void InvalidCommand::process(k4::ServerState& state) const { state.status.set("Usage: %s", usage_); }

const char* a2ui(const char* start, const char* end, unsigned& out) {
  for (out = 0; start < end; ++start) {
    if (*start < '0' || *start > '9') {
      break;
    }
    out = out * 10 + (*start - '0');
  }
  return start;
}

} // namespace detail

const Command* CommandBuffer::parse_impl() {
  using namespace detail;

  if (length_ == 0) {
    return nullptr;
  }
  const char *ptr = buffer_, *next;
  auto skip_ws = [&] {
    while (ptr < end() && *ptr == ' ') {
      ++ptr;
    }
  };
  skip_ws();
  if (length_ >= 2 && ptr[0] == 't' && ptr[1] == 'r') {
    ptr += 2;
    skip_ws();
    unsigned train_no, speed;
    next = a2ui(ptr, end(), train_no);
    if (next == ptr) {
      return invalid<SetTrainSpeedCommand>();
    }
    ptr = next;
    skip_ws();
    next = a2ui(ptr, end(), speed);
    if (next == ptr) {
      return invalid<SetTrainSpeedCommand>();
    }
    ptr = next;
    skip_ws();
    if (ptr != end()) {
      return invalid<SetTrainSpeedCommand>();
    }
    return new (curr_cmd_) SetTrainSpeedCommand{train_no, speed};
  }
  if (length_ >= 2 && ptr[0] == 'r' && ptr[1] == 'v') {
    ptr += 2;
    skip_ws();
    unsigned train_no;
    next = a2ui(ptr, end(), train_no);
    if (next == ptr) {
      return invalid<ReverseTrainCommand>();
    }
    ptr = next;
    skip_ws();
    if (ptr != end()) {
      return invalid<ReverseTrainCommand>();
    }
    return new (curr_cmd_) ReverseTrainCommand{train_no};
  }
  if (length_ >= 2 && ptr[0] == 's' && ptr[1] == 'w') {
    ptr += 2;
    skip_ws();
    unsigned switch_no;
    next = a2ui(ptr, end(), switch_no);
    if (next == ptr) {
      return invalid<ThrowSwitchCommand>();
    }
    ptr = next;
    skip_ws();
    if (ptr == end()) {
      return invalid<ThrowSwitchCommand>();
    }
    bool direction;
    if (ptr[0] == 'S' || ptr[0] == 's') {
      direction = true;
    } else if (ptr[0] == 'C' || ptr[0] == 'c') {
      direction = false;
    } else {
      return invalid<ThrowSwitchCommand>();
    }
    ptr += 1;
    skip_ws();
    if (ptr != end()) {
      return invalid<ThrowSwitchCommand>();
    }
    return new (curr_cmd_) ThrowSwitchCommand{switch_no, direction};
  }
  if (length_ >= 1 && ptr[0] == 'q') {
    ptr += 1;
    skip_ws();
    if (ptr != end()) {
      return invalid<QuitCommand>();
    }
    return new (curr_cmd_) QuitCommand{};
  }
  return new (curr_cmd_) Command{};
}

} // namespace cmd
