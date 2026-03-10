#pragma once
#include "marklin/marklin_message.h"
#include "marklin/marklin_train_track.h"
#include "message.h"
#include "send_util.h"
#include "util/history.h"
#include "util/ring_buffer.h"

namespace k4 {
// UI Utils.
inline void sendSensorHistoryToUI(int uiTid, const History<SensorHistoryEntry, SENSOR_HISTORY_SIZE>& history) {
  UIMsg ui{.type = UIMsgType::RedrawSensors, .sensorHistory{}};
  unsigned idx = 0;
  for (const auto& e : history) {
    ui.sensorHistory.entries[idx++] = e;
  }
  ui.sensorHistory.count = idx;
  notify(uiTid, ui);
}

inline void sendTrainHistoryToUI(int uiTid, const RingBuffer<TrainStatesEntry, TRAIN_HISTORY_SIZE>& trainStates) {
  UIMsg ui{.type = UIMsgType::TrainStates, .trainStates{}};
  unsigned idx = 0;
  for (const auto& e : trainStates) {
    ui.trainStates.entries[idx++] = e;
  }
  ui.trainStates.count = idx;
  notify(uiTid, ui);
}

// Train Track Utils
template <bool forceUpdate = false>
void broadcastSwitchState(int dispatcherTid, int uiTid, marklin::TrainTrackState& ttState, marklin::SwitchId id,
                          marklin::SwitchState state) {

  if (forceUpdate || ttState.getSwitchState(id) != state) {
    ttState.setSwitchState(id, state);
    sendToDispatcher(dispatcherTid, marklin::MMessage::setSwitchState(id, state));
    notify(uiTid, UIMsg{.type = UIMsgType::UpdateSwitch, .switchUpdate{.switchId = id, .state = state}});
  }

  // Adjust central switch pair.
  // TODO: veirfy to ensure this does not violate the ownership rule.
  id = [id] -> marklin::SwitchId {
    switch (id) {
    case 153:
      return 154;
    case 154:
      return 153;
    case 155:
      return 156;
    case 156:
      return 155;
    default:
      return 0;
    }
  }();
  if (id == 0) {
    return;
  }
  state = (state == marklin::SwitchState::Straight ? marklin::SwitchState::Curved : marklin::SwitchState::Straight);
  if (forceUpdate || ttState.getSwitchState(id) != state) {
    ttState.setSwitchState(id, state);
    sendToDispatcher(dispatcherTid, marklin::MMessage::setSwitchState(id, state));
    notify(uiTid, UIMsg{.type = UIMsgType::UpdateSwitch, .switchUpdate{.switchId = id, .state = state}});
  }
}

inline void broadcastTrainSpeedLevel(int dispatcherTid, [[maybe_unused]] int uiTid, marklin::TrainTrackState& ttState,
                                     marklin::TrainId id, marklin::SpeedLevel speedLevel) {
  marklin::Train& train = ttState.getTrain(id);
  train.speedLevel = speedLevel;
  sendToDispatcher(dispatcherTid,
                   marklin::MMessage::setTrainSpeed(id, marklin::convertSpeedLevelToCANSpeed(speedLevel)));
}

// Softreset. Doesn't affect the train speed as it doesn't send the speed message.
inline void resetSystem(int dispatcherTid, int uiTid, marklin::TrainTrackState& ttState) {
  ttState.reset();
  sendToDispatcher(dispatcherTid, marklin::MMessage::systemHaltAll());
  sendToDispatcher(dispatcherTid, marklin::MMessage::systemGoAll());
  for (marklin::SwitchId id = 1; id <= 18; ++id) {
    broadcastSwitchState<true>(dispatcherTid, uiTid, ttState, id, marklin::SwitchState::Straight);
  }
  for (marklin::SwitchId id = 153; id <= 156; ++id) {
    broadcastSwitchState<true>(dispatcherTid, uiTid, ttState, id, marklin::SwitchState::Straight);
  }
}

inline void initTrain(int dispatcherTid, marklin::TrainTrackState& ttState, marklin::TrainId trainId) {
  if (marklin::Train& t = ttState.getTrain(trainId); !t.touched) {
    t.touched = true;
    sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainFunctionState(trainId, marklin::TrainFunction::F0, 1));
    sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainDirection(trainId, marklin::TrainDirection::Forward));
  }
}

inline void setSwitchNodeByLockState(int dispatcherTid, int uiTid, marklin::TrainTrackState& ttState,
                                     const marklin::TrackNode& node) {
  if (node.type != marklin::TrackNode::Type::Branch || !node.lock.hasOwner()) {
    return;
  }
  marklin::SwitchState sw =
      (node.lock.topDirection() == marklin::TrackDirection::Straight ? marklin::SwitchState::Straight
                                                                     : marklin::SwitchState::Curved);
  broadcastSwitchState<false>(dispatcherTid, uiTid, ttState, node.num, sw);
}
} // namespace k4
