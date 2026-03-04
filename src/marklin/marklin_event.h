#pragma once
#include "marklin_message.h"
#include "marklin_train_track.h"

namespace marklin {
struct SensorTriggeredEvent {
  uint32_t id;
  SensorState state;
  SensorTriggeredEvent() = default;
  SensorTriggeredEvent(const MMessage& mmsg)
      : id((uint32_t(mmsg.data[2] << 8) | mmsg.data[3]) - 1), state(SensorState(mmsg.data[5])) {}
};

struct TrainSpeedEvent {
  uint8_t id;
  uint16_t speed;
  TrainSpeedEvent() = default;
  TrainSpeedEvent(const MMessage& mmsg)
      : id(mmsg.data[3]), speed(uint16_t(mmsg.data[4]) << 8 | uint16_t(mmsg.data[5])) {}
};

struct TrainDirectionEvent {
  uint8_t id;
  TrainDirection direction;
  TrainDirectionEvent() = default;
  TrainDirectionEvent(const MMessage& mmsg) : id(mmsg.data[3]), direction(TrainDirection(mmsg.data[4])) {}
};

struct TrainFunctionEvent {
  uint8_t id;
  TrainFunction function;
  uint8_t value;
  TrainFunctionEvent() = default;
  TrainFunctionEvent(const MMessage& mmsg)
      : id(mmsg.data[3]), function(TrainFunction(mmsg.data[4])), value(mmsg.data[5]) {}
};

struct SwitchDirectionEvent {
  uint8_t id;
  SwitchState state;
  bool isSolenoidActivee;
  SwitchDirectionEvent() = default;
  SwitchDirectionEvent(const MMessage& mmsg)
      : id(mmsg.data[3] + 1), state(SwitchState(mmsg.data[4])), isSolenoidActivee(mmsg.data[5]) {}
};

} // namespace marklin
