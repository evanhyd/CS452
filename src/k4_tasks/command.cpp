#include "command.h"

namespace cmd {

namespace {

// Convert ascii string to unsigned int.
const char* a2ui(const char* start, const char* end, unsigned& out) {
  for (out = 0; start < end; ++start) {
    if (*start < '0' || *start > '9') {
      break;
    }
    out = out * 10 + (*start - '0');
  }
  return start;
}

constexpr const char* USAGE_TR = "tr <train number> <train speed> - Set train speed";
constexpr const char* USAGE_RV = "rv <train number> - Reverse train direction";
constexpr const char* USAGE_SW = "sw <switch number> <switch direction> - Set switch to straight (S) or curved (C)";
constexpr const char* USAGE_Q = "q - Quit program and reboot";

} // namespace

ParsedCommand CommandBuffer::parse_impl() {
  if (length_ == 0) {
    return {.tag = CmdTag::None, .empty{}};
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
      return {.tag = CmdTag::Invalid, .invalid{USAGE_TR}};
    }
    ptr = next;
    skip_ws();
    next = a2ui(ptr, end(), speed);
    if (next == ptr) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_TR}};
    }
    ptr = next;
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_TR}};
    }
    return {.tag = CmdTag::SetSpeed, .setSpeed{train_no, speed}};
  }
  if (length_ >= 2 && ptr[0] == 'r' && ptr[1] == 'v') {
    ptr += 2;
    skip_ws();
    unsigned train_no;
    next = a2ui(ptr, end(), train_no);
    if (next == ptr) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_RV}};
    }
    ptr = next;
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_RV}};
    }
    return {.tag = CmdTag::Reverse, .reverse{train_no}};
  }
  if (length_ >= 2 && ptr[0] == 's' && ptr[1] == 'w') {
    ptr += 2;
    skip_ws();
    unsigned switch_no;
    next = a2ui(ptr, end(), switch_no);
    if (next == ptr) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_SW}};
    }
    ptr = next;
    skip_ws();
    if (ptr == end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_SW}};
    }
    bool direction;
    if (ptr[0] == 'S' || ptr[0] == 's') {
      direction = true;
    } else if (ptr[0] == 'C' || ptr[0] == 'c') {
      direction = false;
    } else {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_SW}};
    }
    ptr += 1;
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_SW}};
    }
    return {.tag = CmdTag::SetSwitch, .setSwitch{switch_no, direction}};
  }
  if (length_ >= 1 && ptr[0] == 'q') {
    ptr += 1;
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_Q}};
    }
    return {.tag = CmdTag::Quit, .empty{}};
  }
  return {.tag = CmdTag::None, .empty{}};
}

} // namespace cmd
