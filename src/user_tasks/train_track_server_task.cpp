#include "train_track_server_task.h"

#include "message.h"

#include "kernel/syscalls.h"
#include "marklin/marklin_message.h"
#include "marklin/marklin_pathfinding.h"
#include "marklin/marklin_train_track.h"
#include "send_util.h"
#include "util/debug.h"
#include "util/history.h"
#include "util/ring_buffer.h"
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

void sendTrainHistoryToUI(int uiTid, const RingBuffer<TrainStatesEntry, TRAIN_HISTORY_SIZE>& trainStates) {
  UIMsg ui;
  ui.type = UIMsgType::TrainStates;
  unsigned idx = 0;
  for (const auto& e : trainStates) {
    ui.trainStates.entries[idx++] = e;
  }
  ui.trainStates.count = idx;
  notify(uiTid, ui);
}

// Utils
void initTrain(int dispatcherTid, marklin::TrainTrackState& ttState, marklin::TrainId trainId) {
  if (marklin::Train& t = ttState.getTrain(trainId); !t.touched) {
    t.touched = true;
    sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainFunctionState(trainId, marklin::TrainFunction::F0, 1));
    sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainDirection(trainId, marklin::TrainDirection::Forward));
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
  RingBuffer<TrainStatesEntry, TRAIN_HISTORY_SIZE> trainStates;

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
      marklin::Train& train = ttState.getTrain(trainId);
      if (train.stateMachine.type != marklin::TrainStateMachine::Type::Idle) {
        notifyStatusToUI(uiTid, "Train %u is busy right now.", trainId);
        break;
      }

      // Change speed.
      train.speedLevel = speedLevel;
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
      marklin::Train& train = ttState.getTrain(trainId);
      if (train.stateMachine.type != marklin::TrainStateMachine::Type::Idle) {
        notifyStatusToUI(uiTid, "Train %u is busy right now.", trainId);
        break;
      }

      // Perform reversing.
      train.stateMachine.type = marklin::TrainStateMachine::Type::Reversing;
      train.stateMachine.reversing.countdownTicks = 50 + static_cast<unsigned>(train.speedLevel) * 25;
      sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainSpeed(trainId, 0));
      notifyStatusToUI(uiTid, "Reversing train %u in %u ticks.", trainId, train.stateMachine.reversing.countdownTicks);
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
      // Calculate the error for the previous sensor history.
      //   marklin::Speed estSpeed = ttState.getTrainRef(trackNode->trainOwnerId).estimatedSpeed;
      //   if (!sensorHistory.empty()) {
      //     auto& lastEntry = sensorHistory.back();
      //     // Did we predict this specific sensor (msg.sensorEvent.id)?
      //     if (lastEntry.hasPrediction && lastEntry.predictedId == msg.sensorEvent.id) {

      //       // Calculate Time Error: Actual Time - Predicted Time
      //       lastEntry.timeErrorTicks =
      //           static_cast<int32_t>(currentTicks) - static_cast<int32_t>(lastEntry.predictedTicks);

      //       // Calculate Distance Error: Error = TimeDelta * Speed
      //       lastEntry.distErrorMm = lastEntry.timeErrorTicks * estSpeed / 1000;
      //     }
      //   }
      //   {
      //     // Find the next sensor.
      //     marklin::Distance outDist = 0;
      //     const marklin::TrackNode* nextSensor = marklin::getNextSensor(ttState, *trackNode, outDist);
      //     if (nextSensor != nullptr) {
      //       newEntry.predictedId = nextSensor->id;
      //       newEntry.predictedTicks = currentTicks + static_cast<uint32_t>(outDist) /
      //       static_cast<uint32_t>(estSpeed); newEntry.hasPrediction = true;
      //     }
      //   }

      // Calculate the next sensor history.
      SensorHistoryEntry newEntry{};
      newEntry.event = msg.sensorEvent;
      newEntry.ticks = currentTicks;
      newEntry.hasPrediction = false;
      newEntry.timeErrorTicks = 0;
      newEntry.distErrorMm = 0;
      sensorHistory.push(newEntry);
      sendSensorHistoryToUI(uiTid, sensorHistory);

      // Calibrate train's motion state.
      marklin::TrackNode& triggeredNode = ttState.getTrackNodeById(msg.sensorEvent.id);
      if (triggeredNode.owner != marklin::NO_TRAIN) {
        marklin::calibrateTrain(ttState, triggeredNode, currentTicks);
      }
      break;
    }
    case TrainTrackMsgType::GotoCmd: {
      marklin::TrainId trainId = msg.gotoCmd.trainId;
      marklin::SpeedLevel speedLevel = msg.gotoCmd.speedLevel;
      if (!marklin::isValidTrainId(trainId)) {
        notifyStatusToUI(uiTid, "Invalid train id for goto.");
        break;
      }
      if (!marklin::isValidSpeedLevel(speedLevel)) {
        notifyStatusToUI(uiTid, "Invalid speed level for goto.");
        break;
      }
      marklin::TrackNode* dest = ttState.getTrackNodeByName(msg.gotoCmd.location);
      if (!dest) {
        notifyStatusToUI(uiTid, "Invalid track node name for goto.");
        break;
      }

      // Check busy status.
      initTrain(dispatcherTid, ttState, trainId);
      marklin::Train& train = ttState.getTrain(trainId);
      if (train.stateMachine.type != marklin::TrainStateMachine::Type::Idle) {
        notifyStatusToUI(uiTid, "Train %u is busy right now.", trainId);
        break;
      }

      // Locate the train location by first entering the loop and listen to the sensor.
      if (!marklin::lockAllLoopSensorNodes(ttState, trainId)) {
        notifyStatusToUI(uiTid, "Train %u can not acquire an occupied loop.", trainId);
        break;
      }
      train.stateMachine.type = marklin::TrainStateMachine::Type::Locating;
      train.stateMachine.locating.dest = dest->id;

      // Guide the train to enter the loop.
      static constexpr marklin::SwitchId toCurved[] = {3, 5, 8, 9, 11, 12, 14, 15, 18, 154, 155};
      static constexpr marklin::SwitchId toStraight[] = {1, 2, 4, 6, 7, 10, 13, 16, 17, 153, 156};
      for (marklin::SwitchId id : toCurved) {
        ttState.setSwitchState(id, marklin::SwitchState::Curved);
        sendToDispatcher(dispatcherTid, marklin::MMessage::setSwitchState(id, marklin::SwitchState::Curved));
        sendSwitchToUI(uiTid, id, marklin::SwitchState::Curved);
      }
      for (marklin::SwitchId id : toStraight) {
        ttState.setSwitchState(id, marklin::SwitchState::Straight);
        sendToDispatcher(dispatcherTid, marklin::MMessage::setSwitchState(id, marklin::SwitchState::Straight));
        sendSwitchToUI(uiTid, id, marklin::SwitchState::Straight);
      }
      train.speedLevel = speedLevel;
      sendToDispatcher(dispatcherTid,
                       marklin::MMessage::setTrainSpeed(trainId, marklin::convertSpeedLevelToCANSpeed(speedLevel)));

      notifyStatusToUI(uiTid, "Locating train %u.", trainId);
      break;
    }
    case TrainTrackMsgType::TimerTick: {
      currentTicks = msg.time.ticks;

      for (marklin::TrainId trainId = 1; trainId <= marklin::NUM_TRAINS; ++trainId) {
        marklin::Train& train = ttState.getTrain(trainId);
        if (!train.touched) {
          continue;
        }

        // Update train's state machine.
        switch (train.stateMachine.type) {
        case marklin::TrainStateMachine::Type::Idle: {
          break;
        }
        case marklin::TrainStateMachine::Type::Reversing: {
          --train.stateMachine.reversing.countdownTicks;
          if (train.stateMachine.reversing.countdownTicks == 0) {
            train.stateMachine.type = marklin::TrainStateMachine::Type::Idle;
            train.forward = !train.forward;
            auto reverseDir = train.forward ? marklin::TrainDirection::Backward : marklin::TrainDirection::Forward;
            sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainDirection(trainId, reverseDir));
            sendToDispatcher(dispatcherTid, marklin::MMessage::setTrainSpeed(
                                                trainId, marklin::convertSpeedLevelToCANSpeed(train.speedLevel)));
          }
          break;
        }
        case marklin::TrainStateMachine::Type::Locating: {
          // Waiting for the train to get located.
          if (!train.lastVisitedNode) {
            break;
          }

          // Find the path to the destination.
          marklin::unlockAllLoopSensorNodes(ttState);
          marklin::Distance totalPathDist = 0;
          if (!marklin::planPath(ttState, trainId, train.stateMachine.pathing.dest, totalPathDist)) {
            train.stateMachine.type = marklin::TrainStateMachine::Type::Idle;
            notifyStatusToUI(uiTid, "train %u failed to find any path.", trainId);
            break;
          }

          // Update the switches directions.
          marklin::TrackNode* last = train.lastVisitedNode;
          for (marklin::TrackNode* curr : train.path) {
            if (last->type == marklin::TrackNode::Type::Branch) {
              marklin::SwitchState sw = ttState.getSwitchState(last->num);
              marklin::TrackDirection td = (sw == marklin::SwitchState::Straight ? marklin::Straight : marklin::Curved);
              if (last->edges[td].dest != curr) {
                sw = (sw == marklin::SwitchState::Straight ? marklin::SwitchState::Curved
                                                           : marklin::SwitchState::Straight);
                ttState.setSwitchState(last->num, sw);
                sendToDispatcher(dispatcherTid, marklin::MMessage::setSwitchState(last->num, sw));
                sendSwitchToUI(uiTid, last->num, sw);
              }
            }
            last = curr;
          }

          train.stateMachine.type = marklin::TrainStateMachine::Type::Pathing;
          train.stateMachine.pathing.pathDistance = totalPathDist;
          notifyStatusToUI(uiTid, "train %u found path[%u].", trainId, train.path.size());
          break;
        }
        case marklin::TrainStateMachine::Type::Pathing: {
          // Check if the pathing is completed.
          if (train.path.empty()) {
            train.speedLevel = 0;
            sendToDispatcher(dispatcherTid,
                             marklin::MMessage::setTrainSpeed(trainId, marklin::convertSpeedLevelToCANSpeed(0)));
            train.stateMachine.type = marklin::TrainStateMachine::Type::Idle;
            notifyStatusToUI(uiTid, "train %u reached the last sensor.", trainId);
            break;
          }

          // Update train position.
          train.estimatedOffset += train.estimatedSpeed;

          // Update estimated node position.
          marklin::Distance estOffset = train.estimatedOffset;
          marklin::TrackNode* last = train.lastVisitedNode;
          for (marklin::TrackNode* curr : train.path) {
            marklin::Distance dist = getAdjacentDistance(*last, *curr);
            if (estOffset < dist) {
              break;
            }
            estOffset -= dist;
            train.estimatedNode = curr;
            last = curr;
          }

          // Stop the train if close to safe stopping distance.
          marklin::Distance estimatedRemainingPathDist =
              train.stateMachine.pathing.pathDistance - train.estimatedOffset;
          if (estimatedRemainingPathDist <= marklin::getStoppingDistance(train.speedLevel)) {
            train.speedLevel = 0;
            sendToDispatcher(dispatcherTid,
                             marklin::MMessage::setTrainSpeed(trainId, marklin::convertSpeedLevelToCANSpeed(0)));
            train.stateMachine.type = marklin::TrainStateMachine::Type::Idle;
            notifyStatusToUI(uiTid, "train %u arrived at destination (estimate).", trainId);
          }
        }
        }

        // Log to UI.
        if (currentTicks - lastTrainUIRefreshTicks >= 10 && !trainStates.full()) {
          trainStates.pushBack(TrainStatesEntry{
              .trainId = trainId,
              .lastTrackNode = train.lastVisitedNode,
              .estimatedTrackNode = train.estimatedNode,
              .estimatedOffset = train.estimatedOffset,
              .estimatedSpeed = train.estimatedSpeed,
          });
        }
      }

      // Send the train states to the UI.
      if (currentTicks - lastTrainUIRefreshTicks >= 10) {
        lastTrainUIRefreshTicks = currentTicks;
        sendTrainHistoryToUI(uiTid, trainStates);
        trainStates.clear();
      }
      break;
    }
    default:
      break;
    }
  }
}
} // namespace k4
