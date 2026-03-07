#pragma once
#include "util/debug.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

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

struct TrackNode;

struct TrainState {
  uint8_t speedLevel;            // set by user
  uint32_t estimatedSpeed;       // um/tick, calculated from the sensor
  int32_t estimatedAcceleration; // um/tick^2, calculated from change in speed
  uint32_t positionOffset;       // position offset away from the last triggered track node.
  const TrackNode* lastSensorNode;
  uint32_t lastSensorTicks;

  bool forward;
  enum class MotionState : uint8_t { Idle, Reversing } motionState;
  bool touched;
  unsigned reverseCountdownTicks;

  static constexpr int32_t EWMA_DENOMINATOR = 4;
};
static_assert(std::is_trivially_default_constructible_v<TrainState>);

// Convert speed level to speed.
constexpr uint16_t convertSpeedLevelToCANSpeed(unsigned speedLevel) {
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
  TrackNode* src;
  TrackNode* dest;
  uint32_t dist; // millimeters
};

struct TrackNode {
  enum class Type { None, Sensor, Branch, Merge, Enter, Exit };

  const char* name;
  Type type;
  uint8_t id;         // sensor or switch number
  TrackNode* reverse; // same location, but opposite direction
  TrackEdge edge[2];

  uint8_t trainOwnerId; // the train ID that has exclusive access
};

using TrackSet = std::array<TrackNode, NUM_TRACK>;

constexpr bool isValidSwitchIndex(unsigned id) { return (id >= 1 && id <= 18) || (id >= 153 && id <= 156); }

// Convert switch id to switch array index.
constexpr size_t getSwitchIndex(uint8_t id) {
  if (id >= 1 && id <= 18) {
    return id - 1;
  }
  if (id >= 153 && id <= 156) {
    return 18 + id - 153;
  }
  logError("invalid switch id");
}

constexpr size_t getSensorIndex(uint8_t id) { return id; }

struct SensorName {
  char bank;
  uint8_t number;
};

constexpr uint8_t sensorToId(SensorName s) { return static_cast<uint8_t>((s.bank - 'A') * 16 + s.number - 1); }

constexpr SensorName idToSensor(uint8_t id) { return {static_cast<char>('A' + id / 16), uint8_t(id % 16 + 1)}; }

/**********************************
Train Track Definition
***********************************/
struct TrainTrackState {
private:
  std::array<TrainState, NUM_TRAINS> trains{};
  uint32_t currentTrack;
  TrackSet trackA{};
  TrackSet trackB{};
  std::array<SwitchState, NUM_SWITCHES> switches{};

public:
  TrainTrackState();

  // Return the train state by train id (not index).
  TrainState& getTrainStateRef(uint8_t id);

  // Return the switch state by switch id (not index).
  SwitchState getSwitchState(uint8_t id) const;

  // Set the switch state by switch id (not index).
  void setSwitchState(uint8_t id, SwitchState switchState);

  // Set the current track.
  // 0 - A
  // 1 - B
  void setCurrentTrack(uint8_t trackId);

  // Return the current track.
  TrackSet& getCurrentTrack();

  uint8_t theTrain = 0;
};

} // namespace marklin
