#pragma once
#include "marklin/marklin_def.h"
#include "util/debug.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <source_location>

namespace k4 {
struct TrainTrackServerContext;
} // namespace k4

namespace marklin {

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
  struct TrackNode* src;
  struct TrackNode* dest;
  Distance dist;
};

struct TrackNode {
  enum class Type {
    None,
    Sensor, // Straight
    Branch, // Straight, Curved
    Merge,  // Straight
    Enter,  // Straight
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
Train Definition
***********************************/
enum class TrainDirection { NoChange, Forward, Backward, Reverse };
enum class TrainFunction { HeadLight, BoardingSound, F2, BazzingSound };

class NavigationSystem {
  struct FindingPathTask {
    TrackNodeId dest = 0;
    Distance offset = 0;
    SpeedLevel maxSpeedLevel = 0;
    bool needToReverse = false;
  };

  struct ReversingTask {
    SpeedLevel preReversingSpeedLevel = 0;
  };

public:
  enum class State { Manual, FindingPath, Routed, Reversing };
  State state = State::Manual;
  FindingPathTask findingPathTask{};
  ReversingTask reversingTask{};
  bool isWandering = false;
};

class PathFindingSystem;

class KinematicsSystem {
public:
  enum class State { Lost, Tracked };
  State state = State::Lost;
  TrainDirection direction = TrainDirection::Forward;
  SpeedLevel offlineSpeedLevel = 0;
  Speed offlineSpeed = 0;
  Speed estimatedSpeed = 0;
  struct TrackNode* lastSensor = nullptr;
  Distance lastSensorOffset = 0;
  uint32_t lastSensorTicks = 0;
  struct TrackNode* estimatedNode = nullptr;
  Distance estimatedNodeOffset = 0;

  void triggerSensor(TrackNode& sensor, Distance dS, uint32_t ticks) {
    static constexpr int32_t EWMA_DENOMINATOR = 4;
    if (state == State::Tracked) {
      uint32_t dT = ticks - lastSensorTicks;
      if (dT != 0) {
        Speed v = dS / Speed(dT);
        estimatedSpeed = (estimatedSpeed * (EWMA_DENOMINATOR - 1) + v) / EWMA_DENOMINATOR;
      }
    } else {
      state = State::Tracked;
    }

    lastSensor = &sensor;
    lastSensorOffset = 0;
    lastSensorTicks = ticks;
    estimatedNode = &sensor;
    estimatedNodeOffset = 0;
  }

  void onTick(struct TrainTrackState& ttState);
};

struct SensorPredictionSystem {
  struct TrackNode* sensor = nullptr;
  uint32_t predictedTicks = 0;
  int32_t lastTimeErrorTicks = 0;
  Distance lastDistErrorUm = 0;

  void triggerSensor(TrackNode& triggeredSensor, TrackNode* nextSensor, Distance distToNext, Speed estimatedSpeed,
                     uint32_t currentTicks) {
    if (sensor && sensor->id == triggeredSensor.id) {
      lastTimeErrorTicks = static_cast<int32_t>(predictedTicks) - static_cast<int32_t>(currentTicks);
      lastDistErrorUm = lastTimeErrorTicks * estimatedSpeed;
    } else {
      lastTimeErrorTicks = 0;
      lastDistErrorUm = 0;
    }
    sensor = nextSensor;
    if (sensor && estimatedSpeed > 0) {
      predictedTicks = currentTicks + static_cast<uint32_t>(distToNext / estimatedSpeed);
    } else {
      predictedTicks = 0;
    }
  }
};

struct Train {
  NavigationSystem navigation{};
  KinematicsSystem kinematics{};
  SensorPredictionSystem prediction{};
};

// Convert speed level to CAN speed.
constexpr CANSpeed convertSpeedLevelToCANSpeed(SpeedLevel speedLevel) {
  return static_cast<CANSpeed>(speedLevel > 0 ? 1 + (speedLevel - 1) * 77 : 0);
}

constexpr bool isValidSpeedLevel(SpeedLevel speedLevel) { return speedLevel <= MAX_SPEED_LEVEL; }
constexpr bool isValidTrainId(TrainId id) { return 1 <= id && id <= MAX_TRAIN_ID; }

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
  TrackId getCurrentTrackId() const;

  TrackNode& getTrackNodeById(TrackNodeId id);
  TrackNode* getTrackNodeByName(const char* name);
};

} // namespace marklin
