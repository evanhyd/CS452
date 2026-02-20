#pragma once
#include <cstdint>

namespace marklin {
enum class SwitchState : uint8_t { Curved, Straight };
enum class SensorState : uint8_t { Free, Occupied };
enum class TrainDirection : uint8_t { NoChange, Forward, Backward, Reverse };
enum class TrainFunction : uint8_t {
  F0,
  F1,
  F2,
  F3,
  F4,
  F5,
  F6,
  F7,
  F8,
  F9,
  F10,
  F11,
  F12,
  F13,
  F14,
  F15,
  F16,
  F17,
  F18,
  F19,
  F20,
  F21,
  F22,
  F23,
  F24,
  F25,
  F26,
  F27,
  F28,
  F29,
  F30,
  F31
};

struct TrainState {
  uint16_t speed;
  TrainDirection direction;
  uint8_t functionStates[32];
};

// A local train track that tracks the state of a remote train track.
class TrainTrack {
  TrainState trains[256];
  SwitchState switches[256];
  SensorState sensors[256];

public:
  inline static constexpr uint8_t switchIds[] = {1,  2,  3,  4,  5,  6,  7,  8,   9,   10,  11,
                                                 12, 13, 14, 15, 16, 17, 18, 153, 154, 155, 156};
  inline static constexpr uint8_t sensorIds[] = {
      1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27,
      28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54,
      55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80};

  void setTrainSpeed(uint8_t id, uint16_t speed);
  void setTrainDirection(uint8_t id, TrainDirection direction);
  void setTrainFunctionState(uint8_t id, TrainFunction function, uint8_t state);
  void setSwitchDirection(uint8_t id, SwitchState direction);
  void setSensorState(uint8_t id, SensorState state);

  uint16_t getTrainSpeed(uint8_t id) const;
  TrainDirection getTrainDirection(uint8_t id) const;
  uint8_t getTrainFunctionState(uint8_t id, TrainFunction function) const;
  SwitchState getSwitchState(uint8_t id) const;
  SensorState getSensorState(uint8_t id) const;
};
} // namespace marklin
