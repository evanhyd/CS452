#include "marklin_train_track.h"

namespace marklin {
void TrainTrack::setTrainSpeed(uint8_t id, uint16_t speed) { trains[id].speed = speed; }

void TrainTrack::setTrainDirection(uint8_t id, TrainDirection direction) {
  switch (direction) {
  case TrainDirection::NoChange:
    break;
  case TrainDirection::Forward:
    trains[id].direction = TrainDirection::Forward;
    break;
  case TrainDirection::Backward:
    trains[id].direction = TrainDirection::Backward;
    break;
  case TrainDirection::Reverse:
    trains[id].direction =
        (trains[id].direction == TrainDirection::Forward ? TrainDirection::Backward : TrainDirection::Forward);
    break;
  default:
    break;
  }
}

void TrainTrack::setTrainFunctionState(uint8_t id, TrainFunction function, uint8_t state) {
  trains[id].functionStates[static_cast<size_t>(function)] = state;
}

void TrainTrack::setSwitchDirection(uint8_t id, SwitchState state) { switches[id] = state; }

void TrainTrack::setSensorState(uint8_t id, SensorState state) { sensors[id] = state; }

uint16_t TrainTrack::getTrainSpeed(uint8_t id) const { return trains[id].speed; }

TrainDirection TrainTrack::getTrainDirection(uint8_t id) const { return trains[id].direction; }

uint8_t TrainTrack::getTrainFunctionState(uint8_t id, TrainFunction function) const {
  return trains[id].functionStates[static_cast<size_t>(function)];
}

SwitchState TrainTrack::getSwitchState(uint8_t id) const { return switches[id]; }

SensorState TrainTrack::getSensorState(uint8_t id) const { return sensors[id]; }

} // namespace marklin
