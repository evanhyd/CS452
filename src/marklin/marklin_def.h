#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

namespace marklin {

/**********************************
Type Alias
***********************************/
using Speed = int32_t;
using SpeedLevel = uint16_t;
using CANSpeed = uint16_t;
using Distance = int32_t; // um
inline constexpr Distance INF_DISTANCE = std::numeric_limits<Distance>::max() / 4;
using TrainId = uint8_t;
using SensorNumber = uint8_t; // the "12" in A12
using SwitchId = uint8_t;
using TrackNodeId = uint8_t;  // Track Node array index, maps 1 to 1 to sensor id by coincidence.
using TrackNodeNum = uint8_t; // Map to either a switch id or a sensor id or other special id.
using TrackId = uint32_t;     // Track A (0) or Track B (1)

/**********************************
Magic Constant
***********************************/
inline constexpr size_t MAX_TRAIN_ID = 21;
inline constexpr size_t NUM_TRACK_NODES = 144;
inline constexpr size_t NUM_SWITCHES = 22;
inline constexpr size_t NUM_TRAIN_IN_LAB = 8;
inline constexpr size_t MAX_SPEED_LEVEL = 14;

inline constexpr TrainId NO_TRAIN = 0;

} // namespace marklin
