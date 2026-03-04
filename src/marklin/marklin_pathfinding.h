#pragma once
#include "marklin_train_track.h"

namespace marklin {
inline TrackNode* getNextSensor(TrainTrackState& state, TrackNode* current, int& outDist) {
  outDist = 0;
  TrackNode* node = current;
  while (node != nullptr && node->type != TrackNode::Type::Sensor) {
    if (node->type == TrackNode::Type::Exit) {
      return nullptr;
    }
    size_t dir = TrackDirection::Ahead;
    if (node->type == TrackNode::Type::Branch) {
      auto sw = state.getSwitchState(static_cast<uint8_t>(node->id));
      dir = sw == SwitchState::Straight ? TrackDirection::Straight : TrackDirection::Curved;
    }
    outDist += node->edge[dir].dist;
    node = node->edge[dir].dest;
  }
  return node;
}
} // namespace marklin
