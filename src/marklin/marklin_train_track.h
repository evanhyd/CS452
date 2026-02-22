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
} // namespace marklin
