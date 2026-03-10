#pragma once
#include "util/debug.h"
#include "util/ring_buffer.h"
#include "util/static_stack.h"
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
static constexpr size_t MAX_SPEED_LEVEL = 14;
static constexpr size_t NUM_TRAINS = 60;
static constexpr size_t NUM_TRACK_NODES = 144;
static constexpr size_t NUM_SWITCHES = 22;
static constexpr TrainId NO_TRAIN = 0;

/**********************************
Train Definition
***********************************/
enum class TrainDirection { NoChange, Forward, Backward, Reverse };
enum class TrainFunction { // clang-format off
  F0, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12, F13, F14, F15, F16, F17, F18, F19, F20, F21, F22, F23, F24, F25, F26, F27, F28, F29, F30, F31
}; // clang-format on

struct TrackNode;

struct TrainStateMachine {
  enum class Type { Idle, Reversing, Locating, Pathing } type;
  struct IdleState {};
  struct ReversingState {
    unsigned countdownTicks;
  };
  struct LocatingState {
    TrackNodeId dest;
    Distance offset;
  };
  struct PathingState {
    TrackNodeId dest;
    Distance offset;
    Distance pathDistance;
  };

  union {
    IdleState idle;
    ReversingState reversing;
    LocatingState locating;
    PathingState pathing;
  };
};

struct Train {
  bool touched;
  bool forward;

  TrainStateMachine stateMachine{};

  // Motion State
  SpeedLevel speedLevel;
  Speed estimatedSpeed; // um/tick, calculated from the sensor
  uint32_t lastCalibrateTicks;

  TrackNode* lastVisitedNode;
  Distance estimatedOffsetFromLast; // um away from the last visited node.

  TrackNode* estimatedNode;
  Distance estimatedOffsetFromEstimatedNode; // um away from the estimated node.
  Distance estimatedPathDistance;            // um away form the destination.

  RingBuffer<TrackNode*, NUM_TRACK_NODES> path;

  TrackNode* lastTrippedSensor;
  uint32_t lastTrippedTicks;
  TrackNode* predictedNextSensor;
  uint32_t predictedNextSensorTicks;
  int32_t lastTimeErrorTicks;
  Distance lastDistErrorUm;
};

// Convert speed level to speed.
constexpr CANSpeed convertSpeedLevelToCANSpeed(SpeedLevel speedLevel) {
  return static_cast<CANSpeed>(speedLevel > 0 ? 1 + (speedLevel - 1) * 77 : 0);
}

constexpr bool isValidSpeedLevel(SpeedLevel speedLevel) { return speedLevel <= MAX_SPEED_LEVEL; }
constexpr bool isValidTrainId(TrainId id) { return 1 <= id && id <= NUM_TRAINS; }

/**********************************
Switch Definition
***********************************/
enum class SwitchState { Curved, Straight };

constexpr bool isValidSwitchId(SwitchId id) { return (id >= 1 && id <= 18) || (id >= 153 && id <= 156); }

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
enum TrackDirection { Straight = 0, Curved = 1 };

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
    Sensor, // Straight
    Branch, // Straight, Curved
    Merge,  // Straight
    Enter,  // Ahead
    Exit,   // Straight
  };

  class NodeLock {
  public:
    NodeLock() : owner_(NO_TRAIN), directionStack_() {}

    bool tryAcquire(TrainId trainId, TrackDirection direction = TrackDirection::Straight) {
      if (canAcquire(trainId)) {
        owner_ = trainId;
        directionStack_.push(direction);
        return true;
      }
      return false;
    }

    bool canAcquire(TrainId trainId) const { return directionStack_.empty() || owner_ == trainId; }

    void release(TrainId trainId, std::source_location loc = std::source_location::current()) {
      if (hasOwner() && owner_ == trainId) {
        directionStack_.pop();
        if (directionStack_.empty()) {
          owner_ = NO_TRAIN;
        }
        return;
      }
      logError("release failed", loc);
    }

    TrackDirection topDirection() const {
      if (hasOwner()) {
        return directionStack_.top();
      }
      return TrackDirection::Straight;
    }

    bool hasOwner() const { return !directionStack_.empty(); }

    TrainId owner() const { return owner_; }

  private:
    TrainId owner_;
    StaticStack<TrackDirection, 4> directionStack_;
  };

  const char* name;
  Type type;
  TrackNodeNum num;   // sensor or switch number
  TrackNodeId id;     // TrackSet index
  TrackNode* reverse; // same location, but opposite direction
  TrackEdge edges[2];
  NodeLock lock; // the train ID that has exclusive access
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

  void reset();

  Train& getTrain(TrainId id);
  SwitchState getSwitchState(SwitchId id) const;
  void setSwitchState(SwitchId id, SwitchState switchState);

  // Set the current track (0 - A, 1 - B).
  void setCurrentTrack(TrackId id);

  TrackNode& getTrackNodeById(TrackNodeId id);

  // Perform a linear scan to get track node by its name. Very bad performance.
  TrackNode* getTrackNodeByName(const char* name);
};

} // namespace marklin
