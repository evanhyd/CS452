#pragma once
#include "util/debug.h"
#include "util/ring_buffer.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace marklin {
/**********************************
Type Alias
***********************************/
using Speed = int32_t;
using SpeedLevel = uint16_t;
using CANSpeed = uint16_t;
using Distance = int32_t;
using TrainId = uint8_t;
using SensorNumber = uint8_t; // the "12" in A12
using SwitchId = uint8_t;
using TrackNodeId = uint8_t;  // Track Node array index, maps 1 to 1 to sensor id by coincidence.
using TrackNodeNum = uint8_t; // Map to either a switch id or a sensor id or other special id.
using TrackId = uint8_t;      // Track A or Track B

/**********************************
Magic Constant
***********************************/
static constexpr size_t NUM_TRAINS = 60;
static constexpr size_t NUM_TRACK_NODES = 144;
static constexpr size_t NUM_SWITCHES = 22;

/**********************************
Train Definition
***********************************/
enum class TrainDirection { NoChange, Forward, Backward, Reverse };
enum class TrainFunction { // clang-format off
  F0, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12, F13, F14, F15, F16, F17, F18, F19, F20, F21, F22, F23, F24, F25, F26, F27, F28, F29, F30, F31
}; // clang-format on

struct TrackNode;

struct Train {
  bool touched;
  bool forward;
  enum class MotionState { Idle, Reversing } motionState;
  unsigned reverseCountdownTicks;
  SpeedLevel speedLevel;

  // Path Finding
  Speed estimatedSpeed;     // um/tick, calculated from the sensor
  Distance estimatedOffset; // position offset away from the last triggered track node.
  TrackNodeId lastVisitedNodeId;
  uint32_t lastVisitedTicks;
  RingBuffer<TrackNodeId, NUM_TRACK_NODES> path;
};

// Convert speed level to speed.
constexpr CANSpeed convertSpeedLevelToCANSpeed(SpeedLevel speedLevel) {
  return static_cast<CANSpeed>(speedLevel > 0 ? 1 + (speedLevel - 1) * 77 : 0);
}

/**********************************
Switch Definition
***********************************/
enum class SwitchState { Curved, Straight };

constexpr bool isValidSwitchIndex(SwitchId id) { return (id >= 1 && id <= 18) || (id >= 153 && id <= 156); }

// Convert switch id to switch array index.
constexpr size_t getSwitchIndex(SwitchId id) {
  if (id >= 1 && id <= 18) {
    return id - 1;
  }
  if (id >= 153 && id <= 156) {
    return 18 + id - 153;
  }
  logError("invalid switch id");
}

/**********************************
Sensor Definition
***********************************/
enum class SensorState { Free, Occupied };

struct Sensor {
  char bank;
  SensorNumber number;
};

// Convert Sensor to track node id.
constexpr TrackNodeId sensorToTrackNodeId(Sensor s) {
  return static_cast<SensorNumber>((s.bank - 'A') * 16 + s.number - 1);
}

// Convert track node id to Sensor.
constexpr Sensor trackNodeIdToSensor(TrackNodeId id) {
  return {static_cast<char>('A' + id / 16), uint8_t(id % 16 + 1)};
}

/**********************************
Track Definition
***********************************/
enum TrackDirection { Ahead = 0, Straight = 0, Curved = 1 };

struct TrackEdge {
  TrackEdge* reverse;
  TrackNode* src;
  TrackNode* dest;
  Distance dist; // micro-meters
  int32_t speedCompPercentage;
};

struct TrackNode {
  enum class Type {
    None,
    Sensor, // Ahead
    Branch, // Straight, Curved
    Merge,  // Ahead
    Enter,  // Ahead
    Exit,   // None
  };
  const char* name;
  Type type;
  TrackNodeNum num;   // sensor or switch number
  TrackNodeId id;     // TrackSet index
  TrackNode* reverse; // same location, but opposite direction
  TrackEdge edges[2];
  TrainId trainOwnerId; // the train ID that has exclusive access
};

/**********************************
Train Track Definition
***********************************/
using TrackSet = std::array<TrackNode, NUM_TRACK_NODES>;

struct TrainTrackState {
private:
  std::array<Train, NUM_TRAINS> trains{};
  uint32_t currentTrack;
  TrackSet trackA{};
  TrackSet trackB{};
  std::array<SwitchState, NUM_SWITCHES> switches{};

public:
  TrainTrackState();
  Train& getTrainRef(TrainId id);
  SwitchState getSwitchState(SwitchId id) const;
  void setSwitchState(SwitchId id, SwitchState switchState);

  // Set the current track (0 - A, 1 - B).
  void setCurrentTrack(TrackId id);
  TrackNode& getTrackNodeRef(TrackNodeId id);

  TrainId theTrain = 0;
};

} // namespace marklin
