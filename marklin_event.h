#pragma once
#include "fmt.h"
#include "marklin_message.h"
#include "marklin_train_track.h"

namespace marklin {

template <typename EventType> struct CS2Event {
  void toString(char* buffer) const { static_cast<const EventType&>(*this).toString(buffer); }
};

struct SensorTriggeredEvent : CS2Event<SensorTriggeredEvent> {
  uint8_t id;
  SensorState state;

  SensorTriggeredEvent(const MMessage& message) : id(message.data[3]), state(SensorState(message.data[5])) {}

  void toString(char* buffer) const {
    kit::formatString(buffer, "SensorTriggerEvent - id: %u, state: %u", uint32_t(id), uint32_t(state));
  }
};

struct TrainSpeedEvent : CS2Event<TrainSpeedEvent> {
  uint8_t id;
  uint16_t speed;

  TrainSpeedEvent(const MMessage& message)
      : id(message.data[3]), speed(uint16_t(message.data[4]) << 8 | uint16_t(message.data[5])) {}

  void toString(char* buffer) const {
    kit::formatString(buffer, "TrainSpeedEvent - id: %u, speed: %u", uint32_t(id), uint32_t(speed));
  }
};

struct TrainDirectionEvent : CS2Event<TrainDirectionEvent> {
  uint8_t id;
  TrainDirection direction;

  TrainDirectionEvent(const MMessage& message) : id(message.data[3]), direction(TrainDirection(message.data[4])) {}

  void toString(char* buffer) const {
    kit::formatString(buffer, "TrainDirectionEvent - id: %u, direction: %u", uint32_t(id), uint32_t(direction));
  }
};

struct TrainFunctionEvent : CS2Event<TrainFunctionEvent> {
  uint8_t id;
  TrainFunction function;
  uint8_t value;

  TrainFunctionEvent(const MMessage& message)
      : id(message.data[3]), function(TrainFunction(message.data[4])), value(message.data[5]) {}

  void toString(char* buffer) const {
    kit::formatString(buffer, "TrainFunctionEvent - id: %u, function: %u, value: %u", uint32_t(id), uint32_t(function),
                      uint32_t(value));
  }
};

struct SwitchDirectionEvent : CS2Event<SwitchDirectionEvent> {
  uint8_t id;
  SwitchState state;
  bool isSolenoidActivee;

  SwitchDirectionEvent(const MMessage& message)
      : id(message.data[3] + 1), state(SwitchState(message.data[4])), isSolenoidActivee(message.data[5]) {}

  void toString(char* buffer) const {
    kit::formatString(buffer, "SwitchDirectionEvent - id: %u, direction: %c, solenoid: %u", uint32_t(id),
                      (state == SwitchState::Curved ? 'Y' : 'I'), uint32_t(isSolenoidActivee));
  }
};

} // namespace marklin
