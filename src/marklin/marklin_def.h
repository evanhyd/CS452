#pragma once

#include <cstddef>
#include <cstdint>

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
using TrackId = uint32_t;     // Track A (0) or Track B (1)

/**********************************
Magic Constant
***********************************/
static constexpr size_t MAX_TRAIN_ID = 21;
static constexpr size_t NUM_TRACK_NODES = 144;
static constexpr size_t NUM_SWITCHES = 22;
static constexpr size_t NUM_TRAIN_IN_LAB = 8;
static constexpr size_t MAX_SPEED_LEVEL = 14;

static constexpr TrainId NO_TRAIN = 0;

} // namespace marklin
