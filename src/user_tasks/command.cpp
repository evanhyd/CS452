#include "command.h"
#include "marklin/marklin_train_track.h"

namespace cmd {

namespace {

// Convert ascii string to unsigned int.
const char* a2ui(const char* start, const char* end, unsigned& out) {
  for (out = 0; start < end; ++start) {
    if (*start < '0' || *start > '9') {
      break;
    }
    out = out * 10 + static_cast<unsigned>(*start - '0');
  }
  return start;
}

constexpr const char* USAGE_TR = "tr <train id> <speed level> - Set train speed level";
constexpr const char* USAGE_RV = "rv <train id> - Reverse train direction";
constexpr const char* USAGE_SW = "sw <switch id> <switch direction> - Set switch to straight (S) or curved (C)";
constexpr const char* USAGE_ST = "st <track id> - Set track to A or B";
constexpr const char* USAGE_LOOP = "loop <train id> - Send train to the captive loop";
constexpr const char* USAGE_GOTO = "goto <train id> <location> - Send train to a track node (e.g. A5, BR15)";
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
    unsigned trainId, speedLevel;
    next = a2ui(ptr, end(), trainId);
    if (next == ptr) {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_TR}};
    }
    ptr = next;
    skip_ws();
    next = a2ui(ptr, end(), speedLevel);
    if (next == ptr) {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_TR}};
    }
    ptr = next;
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_TR}};
    }
    return {.tag = CmdTag::SetSpeed, .setSpeedData{marklin::TrainId(trainId), marklin::SpeedLevel(speedLevel)}};
  }
  if (length_ >= 2 && ptr[0] == 'r' && ptr[1] == 'v') {
    ptr += 2;
    skip_ws();
    unsigned trainId;
    next = a2ui(ptr, end(), trainId);
    if (next == ptr) {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_RV}};
    }
    ptr = next;
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_RV}};
    }
    return {.tag = CmdTag::Reverse, .reverseData{marklin::TrainId(trainId)}};
  }
  if (length_ >= 2 && ptr[0] == 's' && ptr[1] == 'w') {
    ptr += 2;
    skip_ws();
    unsigned switchId;
    next = a2ui(ptr, end(), switchId);
    if (next == ptr) {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_SW}};
    }
    ptr = next;
    skip_ws();
    if (ptr == end()) {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_SW}};
    }
    marklin::SwitchState state;
    if (ptr[0] == 'S' || ptr[0] == 's') {
      state = marklin::SwitchState::Straight;
    } else if (ptr[0] == 'C' || ptr[0] == 'c') {
      state = marklin::SwitchState::Curved;
    } else {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_SW}};
    }
    ptr += 1;
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_SW}};
    }
    return {.tag = CmdTag::SetSwitch, .setSwitchData{marklin::SwitchId(switchId), state}};
  }
  if (length_ >= 2 && ptr[0] == 's' && ptr[1] == 't') {
    ptr += 2;
    skip_ws();
    char trackLetter = *ptr;
    if (trackLetter == 'a' || trackLetter == 'A') {
      return {.tag = CmdTag::SetTrack, .setTrackData{0}};
    }
    if (trackLetter == 'b' || trackLetter == 'B') {
      return {.tag = CmdTag::SetTrack, .setTrackData{1}};
    }
    return {.tag = CmdTag::Invalid, .invalidData{USAGE_RV}};
  }
  if (length_ >= 1 && ptr[0] == 'q') {
    ptr += 1;
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_Q}};
    }
    return {.tag = CmdTag::Quit, .empty{}};
  }
  if (length_ >= 4 && ptr[0] == 'l' && ptr[1] == 'o' && ptr[2] == 'o' && ptr[3] == 'p') {
    ptr += 4;
    skip_ws();
    unsigned trainId;
    next = a2ui(ptr, end(), trainId);
    if (next == ptr) {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_LOOP}};
    }
    ptr = next;
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_LOOP}};
    }
    return {.tag = CmdTag::Loop, .loopData{marklin::TrainId(trainId)}};
  }
  if (length_ >= 4 && ptr[0] == 'g' && ptr[1] == 'o' && ptr[2] == 't' && ptr[3] == 'o') {
    ptr += 4;
    skip_ws();
    unsigned trainId;
    next = a2ui(ptr, end(), trainId);
    if (next == ptr) {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_GOTO}};
    }
    ptr = next;
    skip_ws();
    if (ptr == end()) {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_GOTO}};
    }
    const char* locStart = ptr;
    while (ptr < end() && *ptr != ' ') {
      ++ptr;
    }
    size_t locLen = static_cast<size_t>(ptr - locStart);
    if (locLen == 0 || locLen >= 16) {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_GOTO}};
    }
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalidData{USAGE_GOTO}};
    }
    ParsedCommand result{.tag = CmdTag::Goto, .gotoData{}};
    result.gotoData.trainId = marklin::TrainId(trainId);
    for (size_t i = 0; i < locLen; ++i) {
      result.gotoData.location[i] = locStart[i];
    }
    result.gotoData.location[locLen] = '\0';
    return result;
  }
  return {.tag = CmdTag::None, .empty{}};
}

} // namespace cmd
