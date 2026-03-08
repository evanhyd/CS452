#include "train_track_server_task.h"

#include "message.h"

#include "kernel/syscalls.h"
#include "marklin/marklin_message.h"
#include "marklin/marklin_pathfinding.h"
#include "marklin/marklin_train_track.h"
#include "send_util.h"
#include "util/debug.h"
#include "util/history.h"
#include <cstdint>

namespace k4 {

namespace {

// Dispatcher Operations
void sendToDispatcher(int dispatcherTid, const marklin::MMessage& mmsg) {
  notify(dispatcherTid, DispatcherMsg{.type = DispatcherMsgType::SendMsg, .mmsg = mmsg});
}

// UI Operations
void sendSwitchToUI(int uiTid, marklin::SwitchId switchId, marklin::SwitchState state) {
  notify(uiTid, UIMsg{.type = UIMsgType::UpdateSwitch, .switchUpdate{.switchId = switchId, .state = state}});
}

void sendAllSwitchesToUI(int uiTid, marklin::TrainTrackState& ttState) {
  for (marklin::SwitchId i = 1; i <= 18; ++i) {
    sendSwitchToUI(uiTid, i, ttState.getSwitchState(i));
  }
  for (marklin::SwitchId i = 153; i <= 156; ++i) {
    sendSwitchToUI(uiTid, i, ttState.getSwitchState(i));
  }
}

void sendSensorHistoryToUI(int uiTid, const History<SensorHistoryEntry, SENSOR_HISTORY_SIZE>& history) {
  UIMsg ui;
  ui.type = UIMsgType::RedrawSensors;
  unsigned idx = 0;
  for (const auto& e : history) {
    ui.sensorHistory.entries[idx++] = e;
  }
  ui.sensorHistory.count = idx;
  notify(uiTid, ui);
}

void sendTrainHistoryToUI(int uiTid, const History<TrainHistoryEntry, TRAIN_HISTORY_SIZE>& history) {
  UIMsg ui;
  ui.type = UIMsgType::TrainHistory;
  unsigned idx = 0;
  for (const auto& e : history) {
    ui.trainHistory.entries[idx++] = e;
  }
  ui.trainHistory.count = idx;
  notify(uiTid, ui);
}

// Utils
void initTrain(int dispatcherTid, marklin::TrainTrackState& ttState, marklin::TrainId trainId) {
  if (marklin::Train& t = ttState.getTrainRef(trainId); !t.touched) {
    t.touched = true;
    sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainFunctionState(trainId, marklin::TrainFunction::F0, 1));
    sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainDirection(trainId, marklin::TrainDirection::Forward));

    // hardcoded start: between B9/B10 and A5/A6, facing towards A5, so last sensor is B9
    // TODO: remove me, get id form looping instead.
    t.lastVisitedNodeId = marklin::sensorToTrackNodeId({'B', 9});
  }
}

} // namespace

void trainTrackTask() {
  if (::RegisterAs(TRAIN_TRACK_SERVER_NAME) < 0) {
    logError("failed to register");
  }
  int dispatcherTid = ::WhoIs(MARKLIN_DISPATCHER_SERVER_NAME);
  if (dispatcherTid < 0) {
    logError("failed to find dispatcher");
  }
  int uiTid = ::WhoIs(UI_VIEW_SERVER_NAME);
  if (uiTid < 0) {
    logError("failed to find UI server");
  }

  uint32_t currentTicks = 0;
  uint32_t lastTrainUIRefreshTicks = 0;
  marklin::TrainTrackState ttState{};
  History<SensorHistoryEntry, SENSOR_HISTORY_SIZE> sensorHistory;
  History<TrainHistoryEntry, TRAIN_HISTORY_SIZE> trainHistory;

  sendAllSwitchesToUI(uiTid, ttState);

  for (;;) {
    // Receive a train track message.
    TrainTrackMsg msg;
    int senderTid;
    ::Receive(&senderTid, reinterpret_cast<char*>(&msg), sizeof(msg));
    ::Reply(senderTid, "", 0);

    switch (msg.type) {
    case TrainTrackMsgType::SetSpeedCmd: {
      // Check argument ranges.
      marklin::TrainId trainId = msg.setSpeedCmd.trainId;
      marklin::SpeedLevel speedLevel = msg.setSpeedCmd.speedLevel;
      if (!marklin::isValidSpeedLevel(speedLevel)) {
        logError("invalid train speed level");
      }
      if (!marklin::isValidTrainId(trainId)) {
        logError("invalid train id");
      }

      // Check busy status.
      initTrain(dispatcherTid, ttState, trainId);
      marklin::Train& t = ttState.getTrainRef(trainId);
      if (t.action != marklin::Train::Action::Idle) {
        notifyStatusToUI(uiTid, "Train %u is busy right now.", trainId);
        break;
      }

      // Change speed.
      t.speedLevel = speedLevel;
      sendToDispatcher(dispatcherTid,
                       marklin::MMessage::setTrainSpeed(trainId, marklin::convertSpeedLevelToCANSpeed(speedLevel)));
      notifyStatusToUI(uiTid, "Set train %u to speed %u.", trainId, speedLevel);
      break;
    }
    case TrainTrackMsgType::ReverseCmd: {
      marklin::TrainId trainId = msg.reverseCmd.trainId;
      if (!marklin::isValidTrainId(trainId)) {
        logError("invalid train id");
      }

      // Check busy status.
      initTrain(dispatcherTid, ttState, trainId);
      marklin::Train& t = ttState.getTrainRef(trainId);
      if (t.action != marklin::Train::Action::Idle) {
        notifyStatusToUI(uiTid, "Train %u is busy right now.", trainId);
        break;
      }

      // Perform reversing.
      t.action = marklin::Train::Action::Reversing;
      t.reverseCountdownTicks = 50 + static_cast<unsigned>(t.speedLevel) * 25;
      sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainSpeed(trainId, 0));
      notifyStatusToUI(uiTid, "Reversing train %u in %u ticks.", trainId, t.reverseCountdownTicks);
      break;
    }
    case TrainTrackMsgType::SetSwitchCmd: {
      marklin::SwitchId switchId = msg.setSwitchCmd.switchId;
      marklin::SwitchState switchState = msg.setSwitchCmd.state;
      if (!marklin::isValidSwitchId(switchId)) {
        logError("invalid switch id");
      }

      ttState.setSwitchState(switchId, switchState);
      sendToDispatcher(dispatcherTid, marklin::MMessage::setSwitchState(switchId, switchState));
      sendSwitchToUI(uiTid, switchId, switchState);
      break;
    }
    case TrainTrackMsgType::SetTrackCmd: {
      ttState.setCurrentTrack(msg.setTrackCmd.trackId);
      notifyStatusToUI(uiTid, "Switched track to %c.", (msg.setTrackCmd.trackId == 0 ? 'A' : 'B'));
      break;
    }
    case TrainTrackMsgType::SensorEvent: {
      if (msg.sensorEvent.state != marklin::SensorState::Occupied) {
        break;
      }

      // Get the train track and the owner train info.
      marklin::TrackNodeId trackNodeId = msg.sensorEvent.id;
      marklin::TrackNode* trackNode = &ttState.getTrackNodeRef(trackNodeId);

      // Calculate the error for the previous sensor history.
      marklin::Speed estSpeed = ttState.getTrainRef(trackNode->trainOwnerId).estimatedSpeed;
      if (!sensorHistory.empty()) {
        auto& lastEntry = sensorHistory.back();
        // Did we predict this specific sensor (msg.sensorEvent.id)?
        if (lastEntry.hasPrediction && lastEntry.predictedId == msg.sensorEvent.id) {

          // Calculate Time Error: Actual Time - Predicted Time
          lastEntry.timeErrorTicks =
              static_cast<int32_t>(currentTicks) - static_cast<int32_t>(lastEntry.predictedTicks);

          // Calculate Distance Error: Error = TimeDelta * Speed
          lastEntry.distErrorMm = lastEntry.timeErrorTicks * estSpeed / 1000;
        }
      }

      // Calculate the next sensor history.
      SensorHistoryEntry newEntry{};
      newEntry.event = msg.sensorEvent;
      newEntry.ticks = currentTicks;
      newEntry.hasPrediction = false;
      newEntry.timeErrorTicks = 0;
      newEntry.distErrorMm = 0;
      {
        // Find the next sensor.
        marklin::Distance outDist = 0;
        const marklin::TrackNode* nextSensor = marklin::getNextSensor(ttState, *trackNode, outDist);
        if (nextSensor != nullptr) {
          newEntry.predictedId = nextSensor->id;
          newEntry.predictedTicks = currentTicks + static_cast<uint32_t>(outDist) / static_cast<uint32_t>(estSpeed);
          newEntry.hasPrediction = true;
        }
      }
      sensorHistory.push(newEntry);
      sendSensorHistoryToUI(uiTid, sensorHistory);

      // Recalibrate train's motion state.
      marklin::calibrateTrainFromTrackNode(ttState, msg.sensorEvent.id, currentTicks);
      break;
    }
    case TrainTrackMsgType::GotoCmd: {
      uint8_t trainId = msg.gotoCmd.trainId;
      if (trainId == 0 || trainId > marklin::NUM_TRAINS) {
        notifyStatusToUI(uiTid, "Invalid train id for goto.");
        break;
      }
      initTrain(dispatcherTid, ttState, trainId);

      marklin::Train& train = ttState.getTrainRef(trainId);
      if (train.action != marklin::Train::Action::Idle) {
        notifyStatusToUI(uiTid, "Please wait until train finishes the current action.");
        break;
      }

      // Enter the loop.

      //   const marklin::TrackNode* startNode = train.lastVisitedNodeId;
      //   if (startNode == nullptr) {
      //     notifyStatusToUI(uiTid, "Train %u has no known location.", trainId);
      //     break;
      //   }

      //   const marklin::TrackNode* destNode = findNodeByName(ttState, msg.gotoCmd.location);
      //   if (destNode == nullptr) {
      //     notifyStatusToUI(uiTid, "Unknown track node '%s'.", msg.gotoCmd.location);
      //     break;
      //   }

      //   uint32_t destId = destNode->id;
      //   auto isGoal = [destId](const marklin::TrackNode* node) { return node->id == destId; };
      //   const marklin::TrackNode* targetNode = bfsAndSetSwitches(startNode, isGoal, ttState, dispatcherTid, uiTid);
      //   if (targetNode == nullptr) {
      //     notifyStatusToUI(uiTid, "No path found from %s to %s.", startNode->name, destNode->name);
      //   } else {
      //     notifyStatusToUI(uiTid, "Path set from %s to %s.", startNode->name, targetNode->name);
      //   }
      break;
    }
    case TrainTrackMsgType::TimerTick: {
      currentTicks = msg.time.ticks;

      for (marklin::TrainId trainId = 1; trainId <= marklin::NUM_TRAINS; ++trainId) {
        marklin::Train& train = ttState.getTrainRef(trainId);
        if (!train.touched) {
          continue;
        }

        // Perform action every tick.
        switch (train.action) {
        case marklin::Train::Action::Idle: {
          break;
        }
        case marklin::Train::Action::Reversing: {
          --train.reverseCountdownTicks;
          if (train.reverseCountdownTicks == 0) {
            train.action = marklin::Train::Action::Idle;
            train.forward = !train.forward;
            auto reverseDir = train.forward ? marklin::TrainDirection::Backward : marklin::TrainDirection::Forward;
            sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainDirection(trainId, reverseDir));
            sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainSpeed(
                                                trainId, marklin::convertSpeedLevelToCANSpeed(train.speedLevel)));
          }
          break;
        }
        case marklin::Train::Action::Looping: {
          // TDOO: implement me
          break;
        }
        }

        // Update the train position
        train.estimatedOffset += train.estimatedSpeed;
        if (currentTicks - lastTrainUIRefreshTicks >= 10 && trainHistory.size() < TRAIN_HISTORY_SIZE) {
          trainHistory.push(TrainHistoryEntry{
              .trainId = trainId,
              .lastTrackNodeId = train.lastVisitedNodeId,
              .estimatedTrackNodeId = train.lastVisitedNodeId,
              .estimatedOffset = train.estimatedOffset,
              .estimatedSpeed = train.estimatedSpeed,
          });
        }
      }

      // Send the train states to the UI.
      if (currentTicks - lastTrainUIRefreshTicks >= 10) {
        lastTrainUIRefreshTicks = currentTicks;
        sendTrainHistoryToUI(uiTid, trainHistory);
      }
      break;
    }
    default:
      break;
    }
  }
}
} // namespace k4
