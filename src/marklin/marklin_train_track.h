#pragma once
#include "util/debug.h"
#include "util/ring_buffer.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <source_location>

namespace marklin {
/**********************************
Type Alias
***********************************/
using Speed = int32_t;
using SpeedLevel = uint16_t;
using CANSpeed = uint16_t;
using Distance = int32_t; // um
using TrainId = uint8_t;
using SensorNumber = uint8_t; // the "12" in A12
using SwitchId = uint8_t;
using TrackNodeId = uint8_t;  // Track Node array index, maps 1 to 1 to sensor id by coincidence.
using TrackNodeNum = uint8_t; // Map to either a switch id or a sensor id or other special id.
using TrackId = uint8_t;      // Track A (0) or Track B (1)

/**********************************
Magic Constant
***********************************/
static constexpr size_t MAX_SPEED_LEVEL = 14;
static constexpr size_t MAX_TRAIN_ID = 56;
static constexpr size_t NUM_TRAIN_IN_LAB = 6;
static constexpr size_t MAX_NODE_PER_TRAIN = 24;
static constexpr size_t NUM_TRACK_NODES = 144;
static constexpr size_t NUM_SWITCHES = 22;
static constexpr TrainId NO_TRAIN = 0;

/**********************************
Train Definition
***********************************/
enum class TrainDirection { NoChange, Forward, Backward, Reverse };
enum class TrainFunction { HeadLight, BoardingSound, F2, BazzingSound };
enum class KinematicState { Lost, Tracked };
enum class NavigationState { Manual, Routing, Halting, Reversing, Yielding };

struct TrainHardware {
  bool forward = true;
  SpeedLevel speedLevel = 0;
  Speed offlineSpeed = 0;

  void reset() {
    forward = true;
    speedLevel = 0;
    offlineSpeed = 0;
  }
};

struct TrainKinematics {
  Speed estimatedSpeed = 0;
  uint32_t lastKnownTicks = 0;
  struct TrackNode* lastKnownNode = nullptr;
  Distance estimatedOffsetFromLast = 0;
  struct TrackNode* estimatedNode = nullptr;
  Distance estimatedOffsetFromEstimatedNode = 0;

  void updateSpeed(Distance dS, uint32_t dT) {
    static constexpr int32_t EWMA_DENOMINATOR = 4;
    if (dT == 0) {
      return;
    }
    Speed v = dS / Speed(dT);
    estimatedSpeed = (estimatedSpeed * (EWMA_DENOMINATOR - 1) + v) / EWMA_DENOMINATOR;
  }

  void reset() {
    estimatedSpeed = 0;
    lastKnownTicks = 0;
    lastKnownNode = nullptr;
    estimatedOffsetFromLast = 0;
    estimatedNode = nullptr;
    estimatedOffsetFromEstimatedNode = 0;
  }
};

struct TrainPrediction {
  TrackNode* predictedNextSensor = nullptr;
  uint32_t predictedNextSensorTicks = 0;
  int32_t lastTimeErrorTicks = 0;
  Distance lastDistErrorUm = 0;

  void reset() {
    predictedNextSensor = nullptr;
    predictedNextSensorTicks = 0;
    lastTimeErrorTicks = 0;
    lastDistErrorUm = 0;
  }
};

struct TrainNavigation {
  TrackNodeId dest = 0;
  Distance offset = 0;
  Distance pathDistance = 0;
  Distance estimatedPathDistance = 0;
  unsigned reverseCountdownTicks = 0;
  marklin::SpeedLevel resumeSpeed = 0;
  marklin::NavigationState resumeNavigationState = NavigationState::Manual;
  int nodeAheadReserved = 0;
  int nodeAheadLocked = 0;
  RingBuffer<TrackNode*, NUM_TRACK_NODES> path;

  void reset() {
    dest = 0;
    offset = 0;
    pathDistance = 0;
    estimatedPathDistance = 0;
    reverseCountdownTicks = 0;
    resumeSpeed = 0;
    resumeNavigationState = NavigationState::Manual;
    nodeAheadReserved = 0;
    nodeAheadLocked = 0;
    path.clear();
  }
};

struct Train {
  KinematicState kinematicState = KinematicState::Lost;
  NavigationState navigationState = NavigationState::Manual;

  TrainHardware hw;
  TrainKinematics kinematics;
  TrainPrediction prediction;
  TrainNavigation nav;

  void reset() {
    kinematicState = KinematicState::Lost;
    navigationState = NavigationState::Manual;
    hw.reset();
    kinematics.reset();
    prediction.reset();
    nav.reset();
  }
};

// Convert speed level to CAN speed.
constexpr CANSpeed convertSpeedLevelToCANSpeed(SpeedLevel speedLevel) {
  return static_cast<CANSpeed>(speedLevel > 0 ? 1 + (speedLevel - 1) * 77 : 0);
}

constexpr bool isValidSpeedLevel(SpeedLevel speedLevel) { return speedLevel <= MAX_SPEED_LEVEL; }
constexpr bool isValidTrainId(TrainId id) { return 1 <= id && id <= MAX_TRAIN_ID; }
void assertTrainState(Train& train, std::source_location loc = std::source_location::current());

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
  Distance dist;
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

  const char* name;
  Type type;
  TrackNodeNum num;   // sensor or switch number
  TrackNodeId id;     // TrackSet index
  TrackNode* reverse; // same location, but opposite direction
  TrackEdge edges[2];
};

/**********************************
Train Track Definition
***********************************/
using TrackSet = std::array<TrackNode, NUM_TRACK_NODES>;

constexpr bool isValidTrack(TrackId id) { return id == 0 || id == 1; };

struct TrainTrackState {
private:
  uint32_t currentTrack;
  std::array<Train, MAX_TRAIN_ID> trains{};
  TrackSet trackA{};
  TrackSet trackB{};
  std::array<SwitchState, NUM_SWITCHES> switches{};

public:
  TrainTrackState();

  void reset();

  Train& getTrain(TrainId id);
  SwitchState getSwitchState(SwitchId id) const;
  void setSwitchState(SwitchId id, SwitchState switchState);
  void setCurrentTrack(TrackId id);

  TrackNode& getTrackNodeById(TrackNodeId id);
  TrackNode* getTrackNodeByName(const char* name);
};

} // namespace marklin
