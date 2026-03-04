#pragma once
#include "util/debug.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace marklin {
/**********************************
Train Definition
***********************************/
static constexpr size_t NUM_TRAINS = 60;
enum class TrainDirection : uint8_t { NoChange, Forward, Backward, Reverse };
// clang-format off
enum class TrainFunction : uint8_t {
  F0, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12, F13, F14, F15, F16, F17, F18, F19, F20, F21, F22, F23, F24, F25, F26, F27, F28, F29, F30, F31 };
// clang-format on

struct TrainState {
  uint8_t speed;
  bool forward;
  enum class State : uint8_t { Idle, Reversing } state;
  bool touched;
  unsigned reverseCountdownTicks;
};

// Convert speed level to speed.
constexpr uint16_t convertSpeedLevelToSpeed(unsigned speedLevel) {
  return static_cast<uint16_t>(speedLevel > 0 ? 1 + (speedLevel - 1) * 77 : 0);
}

/**********************************
Track Definition
***********************************/
static constexpr size_t NUM_TRACK = 144;
static constexpr size_t NUM_SWITCHES = 22;
enum class SwitchState : uint8_t { Curved, Straight };
enum class SensorState : uint8_t { Free, Occupied };
enum TrackDirection : size_t { Ahead = 0, Straight = 0, Curved = 1 };

struct TrackEdge {
  TrackEdge* reverse;
  struct TrackNode* src;
  struct TrackNode* dest;
  int dist; // millimeters
};

struct TrackNode {
  enum class Type { None, Sensor, Branch, Merge, Enter, Exit };

  const char* name;
  Type type;
  uint32_t id;        // sensor or switch number
  TrackNode* reverse; // same location, but opposite direction
  TrackEdge edge[2];
};

using TrackSet = std::array<TrackNode, NUM_TRACK>;

constexpr bool isValidSwitchIndex(uint32_t id) { return (id >= 1 && id <= 18) || (id >= 153 && id <= 156); }

// Convert switch id to switch array index.
constexpr size_t getSwitchIndex(uint32_t id) {
  if (id >= 1 && id <= 18) {
    return id - 1;
  }
  if (id >= 153 && id <= 156) {
    return 18 + id - 153;
  }
  logError("invalid switch id");
}

// Convert sensor id to switch array index.
constexpr size_t getSensorIndex(uint32_t id) { return id; }

/**********************************
Train Track Definition
***********************************/
struct TrainTrackState {
  std::array<TrainState, NUM_TRAINS + 1> trains{};

  int currentTrack;
  TrackSet trackA{};
  TrackSet trackB{};
  std::array<SwitchState, NUM_SWITCHES> switches{};

  TrainTrackState();

  // Return the switch state by switch id (not index).
  SwitchState getSwitchState(uint8_t id) const;

  // Set the switch state by switch id (not index).
  void setSwitchState(uint8_t id, SwitchState switchState);

  // Set the current track.
  // 0 - A
  // 1 - B
  void setCurrentTrack(int trackId);

  // Return the current track.
  TrackSet& getCurrentTrack();
};

} // namespace marklin
