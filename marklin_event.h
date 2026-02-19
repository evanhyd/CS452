#pragma once
#include "ctfmt.h"
#include "fmt.h"
#include "marklin_message.h"
#include "marklin_train_track.h"
#include <utility>

namespace marklin {

template <typename EventType> struct MarklinEvent {
  void toString(char* buffer) const { static_cast<const EventType&>(*this).toString(buffer); }
};

struct SensorTriggeredEvent : MarklinEvent<SensorTriggeredEvent> {
  uint8_t id;
  SensorState state;

  SensorTriggeredEvent(const MMessage& message) : id(message.data[3]), state(SensorState(message.data[5])) {}

  void toString(char* buffer) const {
    kit::formatString_old(buffer, "SensorTriggerEvent - id: %u, state: %u", id, std::to_underlying(state));
  }
};

struct TrainSpeedEvent : MarklinEvent<TrainSpeedEvent> {
  uint8_t id;
  uint16_t speed;

  TrainSpeedEvent(const MMessage& message)
      : id(message.data[3]), speed(uint16_t(message.data[4]) << 8 | uint16_t(message.data[5])) {}

  void toString(char* buffer) const { kit::formatString_old(buffer, "TrainSpeedEvent - id: %u, speed: %u", id, speed); }
};

struct TrainDirectionEvent : MarklinEvent<TrainDirectionEvent> {
  uint8_t id;
  TrainDirection direction;

  TrainDirectionEvent(const MMessage& message) : id(message.data[3]), direction(TrainDirection(message.data[4])) {}

  void toString(char* buffer) const {
    kit::formatString_old(buffer, "TrainDirectionEvent - id: %u, direction: %u", id, std::to_underlying(direction));
  }
};

struct TrainFunctionEvent : MarklinEvent<TrainFunctionEvent> {
  uint8_t id;
  TrainFunction function;
  uint8_t value;

  TrainFunctionEvent(const MMessage& message)
      : id(message.data[3]), function(TrainFunction(message.data[4])), value(message.data[5]) {}

  void toString(char* buffer) const {
    kit::formatString_old(buffer, "TrainFunctionEvent - id: %u, function: %u, value: %u", id,
                          std::to_underlying(function), value);
  }
};

struct SwitchStateEvent : MarklinEvent<SwitchStateEvent> {
  uint8_t id;
  SwitchState state;
  bool isSolenoidActivee;

  SwitchStateEvent(const MMessage& message)
      : id(message.data[3] + 1), state(SwitchState(message.data[4])), isSolenoidActivee(message.data[5]) {}

  void toString(char* buffer) const {
    kit::formatString_old(buffer, "SwitchDirectionEvent - id: %u, direction: %c, solenoid: %u", id,
                          (state == SwitchState::Curved ? 'Y' : 'I'), isSolenoidActivee);
  }
};

} // namespace marklin
