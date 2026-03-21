#pragma once
#include "marklin/marklin_message.h"
#include "marklin/marklin_train_track.h"
#include "message.h"
#include "send_util.h"
#include "train_track_server_context.h"
#include "util/kit_algorithm.h"

namespace k4 {
// UI Utils.
inline void sendSensorHistoryToUI(TrainTrackServerContext& context) {
  UIMsg ui{.type = UIMsgType::RedrawSensors, .sensorHistory{}};
  unsigned idx = 0;
  for (const auto& e : context.sensorHistory) {
    ui.sensorHistory.entries[idx++] = e;
  }
  ui.sensorHistory.count = idx;
  notify(context.uiTid, ui);
}

inline void sendTrainHistoryToUI(TrainTrackServerContext& context) {
  UIMsg ui{.type = UIMsgType::TrainStates, .trainStates{}};
  unsigned idx = 0;
  for (const auto& e : context.trainStates) {
    ui.trainStates.entries[idx++] = e;
  }
  ui.trainStates.count = idx;
  notify(context.uiTid, ui);
}

inline void sendSwitchToUI(TrainTrackServerContext& context, marklin::SwitchId id, marklin::SwitchState state) {
  notify(context.uiTid, UIMsg{.type = UIMsgType::UpdateSwitch, .switchUpdate{.switchId = id, .state = state}});
}

// Train Track Utils
template <bool forceUpdate = false>
void broadcastSwitchState(TrainTrackServerContext& context, marklin::SwitchId id, marklin::SwitchState state) {

  // If the switch is already in the desired state, do nothing
  if (!forceUpdate && context.ttState.getSwitchState(id) == state) {
    return;
  }

  // Identify if this is a center switch and find its partner
  marklin::SwitchId pairingId = [id] -> marklin::SwitchId {
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

  // Straight up the pairing switch first.
  if (pairingId != 0 && state == marklin::SwitchState::Curved) {
    if (forceUpdate || context.ttState.getSwitchState(pairingId) == marklin::SwitchState::Curved) {
      context.ttState.setSwitchState(pairingId, marklin::SwitchState::Straight);
      sendToDispatcher(context.dispatcherTid,
                       marklin::MMessage::setSwitchState(pairingId, marklin::SwitchState::Straight));
      sendSwitchToUI(context, pairingId, marklin::SwitchState::Straight);
    }
  }

  context.ttState.setSwitchState(id, state);
  sendToDispatcher(context.dispatcherTid, marklin::MMessage::setSwitchState(id, state));
  sendSwitchToUI(context, id, state);
}

template <bool forceUpdate = false>
inline void broadcastTrainSpeedLevel(TrainTrackServerContext& context, marklin::TrainId id,
                                     marklin::SpeedLevel speedLevel) {
  marklin::Train& train = context.ttState.getTrain(id);
  if (forceUpdate || train.hw.speedLevel != speedLevel) {
    train.hw.speedLevel = speedLevel;
    sendToDispatcher(context.dispatcherTid,
                     marklin::MMessage::setTrainSpeed(id, marklin::convertSpeedLevelToCANSpeed(speedLevel)));
  }
}

inline void initTrain(TrainTrackServerContext& context, marklin::TrainId id) {
  if (kit::contains(context.activeTrains.begin(), context.activeTrains.end(), id)) {
    return;
  }
  context.activeTrains.pushBack(id);
  sendToDispatcher(context.dispatcherTid,
                   marklin::MMessage::setTrainFunctionState(id, marklin::TrainFunction::HeadLight, 1));
  sendToDispatcher(context.dispatcherTid, marklin::MMessage::setTrainDirection(id, marklin::TrainDirection::Forward));
}

// Soft reset.
inline void resetContext(TrainTrackServerContext& context) {
  context.ttState.reset();
  context.pfSystem.reset();

  // Reset the train states.
  for (marklin::TrainId id : context.activeTrains) {
    context.ttState.getTrain(id).reset();
    broadcastTrainSpeedLevel<true>(context, id, 0);
  }
  context.activeTrains.clear();

  sendToDispatcher(context.dispatcherTid, marklin::MMessage::systemHaltAll());
  sendToDispatcher(context.dispatcherTid, marklin::MMessage::systemGoAll());

  // Reset the switches
  for (marklin::SwitchId id = 1; id <= 18; ++id) {
    broadcastSwitchState<true>(context, id, marklin::SwitchState::Straight);
  }
  for (marklin::SwitchId id = 153; id <= 156; ++id) {
    broadcastSwitchState<true>(context, id, marklin::SwitchState::Straight);
  }
}

} // namespace k4
