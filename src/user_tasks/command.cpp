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
constexpr const char* USAGE_GOTO =
    "goto <train id> <speed level> <location> <offset> - Send train to a track node (e.g. A5, BR15)";
constexpr const char* USAGE_HIT = "hit <sensor> - Simulate sensor hit (e.g. hit A5)";
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
      return {.tag = CmdTag::Invalid, .invalid{USAGE_TR}};
    }
    ptr = next;
    skip_ws();
    next = a2ui(ptr, end(), speedLevel);
    if (next == ptr) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_TR}};
    }
    ptr = next;
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_TR}};
    }
    return {.tag = CmdTag::SetSpeed, .setSpeed{marklin::TrainId(trainId), marklin::SpeedLevel(speedLevel)}};
  }
  if (length_ >= 2 && ptr[0] == 'r' && ptr[1] == 'v') {
    ptr += 2;
    skip_ws();
    unsigned trainId;
    next = a2ui(ptr, end(), trainId);
    if (next == ptr) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_RV}};
    }
    ptr = next;
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_RV}};
    }
    return {.tag = CmdTag::Reverse, .reverse{marklin::TrainId(trainId)}};
  }
  if (length_ >= 2 && ptr[0] == 's' && ptr[1] == 'w') {
    ptr += 2;
    skip_ws();
    unsigned switchId;
    next = a2ui(ptr, end(), switchId);
    if (next == ptr) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_SW}};
    }
    ptr = next;
    skip_ws();
    if (ptr == end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_SW}};
    }
    marklin::SwitchState state;
    if (ptr[0] == 'S' || ptr[0] == 's') {
      state = marklin::SwitchState::Straight;
    } else if (ptr[0] == 'C' || ptr[0] == 'c') {
      state = marklin::SwitchState::Curved;
    } else {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_SW}};
    }
    ptr += 1;
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_SW}};
    }
    return {.tag = CmdTag::SetSwitch, .setSwitch{marklin::SwitchId(switchId), state}};
  }
  if (length_ >= 2 && ptr[0] == 's' && ptr[1] == 't') {
    ptr += 2;
    skip_ws();
    char trackLetter = *ptr;
    if (trackLetter == 'a' || trackLetter == 'A') {
      return {.tag = CmdTag::SetTrack, .setTrack{0}};
    }
    if (trackLetter == 'b' || trackLetter == 'B') {
      return {.tag = CmdTag::SetTrack, .setTrack{1}};
    }
    return {.tag = CmdTag::Invalid, .invalid{USAGE_ST}};
  }
  if (length_ >= 1 && ptr[0] == 'q') {
    ptr += 1;
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_Q}};
    }
    return {.tag = CmdTag::Quit, .empty{}};
  }
  if (length_ >= 4 && ptr[0] == 'g' && ptr[1] == 'o' && ptr[2] == 't' && ptr[3] == 'o') {
    ptr += 4;
    skip_ws();

    unsigned trainId;
    unsigned speedLevel;

    // 1. Parse Train ID
    next = a2ui(ptr, end(), trainId);
    if (next == ptr) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_GOTO}};
    }
    ptr = next;
    skip_ws();

    // 2. Parse Speed Level
    if (ptr == end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_GOTO}};
    }
    next = a2ui(ptr, end(), speedLevel);
    if (next == ptr) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_GOTO}};
    }
    ptr = next;
    skip_ws();

    // 3. Parse Location String
    if (ptr == end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_GOTO}};
    }
    const char* locStart = ptr;
    while (ptr < end() && *ptr != ' ') {
      ++ptr;
    }
    size_t locLen = static_cast<size_t>(ptr - locStart);

    // Validate location length.
    if (locLen == 0 || locLen >= 8) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_GOTO}};
    }
    skip_ws();

    // 4. Parse Offset
    if (ptr == end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_GOTO}};
    }
    int sign = 1;
    if (*ptr == '-') {
      sign = -1;
      ++ptr;
    } else if (*ptr == '+') {
      ++ptr;
    }
    unsigned offsetMag;
    next = a2ui(ptr, end(), offsetMag);
    if (next == ptr) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_GOTO}};
    }
    marklin::Distance offsetMm = sign * static_cast<int32_t>(offsetMag);
    ptr = next;

    // 5. Ensure no extra garbage parameters exist after offset
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_GOTO}};
    }

    ParsedCommand result{.tag = CmdTag::Goto, .gotoData{}};
    result.gotoData.trainId = marklin::TrainId(trainId);
    result.gotoData.speedLevel = marklin::SpeedLevel(speedLevel);
    result.gotoData.offsetMm = offsetMm;
    for (size_t i = 0; i < locLen; ++i) {
      result.gotoData.location[i] = locStart[i];
    }
    result.gotoData.location[locLen] = '\0';
    return result;
  }
  if (length_ >= 3 && ptr[0] == 'h' && ptr[1] == 'i' && ptr[2] == 't') {
    ptr += 3;
    skip_ws();
    if (ptr == end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_HIT}};
    }

    char bank = *ptr;
    if (bank >= 'a' && bank <= 'z') {
      bank = static_cast<char>(bank - ('a' - 'A'));
    }
    if (bank < 'A' || bank > 'E') {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_HIT}};
    }
    ++ptr;

    unsigned sensorNumber;
    next = a2ui(ptr, end(), sensorNumber);
    if (next == ptr || sensorNumber < 1 || sensorNumber > 16) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_HIT}};
    }
    ptr = next;
    skip_ws();
    if (ptr != end()) {
      return {.tag = CmdTag::Invalid, .invalid{USAGE_HIT}};
    }

    marklin::Sensor sensor{.bank = bank, .number = static_cast<marklin::SensorNumber>(sensorNumber)};
    return {.tag = CmdTag::SimulateSensor, .simulateSensor{marklin::sensorToTrackNodeId(sensor)}};
  }
  return {.tag = CmdTag::None, .empty{}};
}

} // namespace cmd
