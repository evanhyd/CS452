#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace marklin {
enum class SwitchState : uint8_t { Curved, Straight };
enum class SensorState : uint8_t { Free, Occupied };
enum class TrainDirection : uint8_t { NoChange, Forward, Backward, Reverse };
enum class TrainFunction : uint8_t {
  // clang-format off
  F0, F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12, F13, F14, F15, F16,
  F17, F18, F19, F20, F21, F22, F23, F24, F25, F26, F27, F28, F29, F30, F31,
  // clang-format on
};

enum TrackDirection : size_t {
  Ahead = 0,
  Straight = 0,
  Curved = 1,
};

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
  int num;            // sensor or switch number
  TrackNode* reverse; // same location, but opposite direction
  TrackEdge edge[2];
};

static constexpr size_t TRACK_MAX = 144;
using TrackSet = std::array<TrackNode, TRACK_MAX>;

void initTrackA(TrackSet& tracks);
void initTrackB(TrackSet& tracks);

} // namespace marklin
