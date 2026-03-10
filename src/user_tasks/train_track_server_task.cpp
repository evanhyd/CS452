#include "train_track_server_task.h"

#include "train_track_util.h"

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

namespace {} // namespace

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

  resetSystem(dispatcherTid, uiTid, ttState);

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
      broadcastTrainSpeedLevel(dispatcherTid, uiTid, ttState, trainId, speedLevel);
      notifyStatusToUI(uiTid, "Set train %u to speed %u.", trainId, speedLevel);
      break;
    }
    case TrainTrackMsgType::ReverseCmd: {
      // Check argument ranges.
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
      broadcastSwitchState(dispatcherTid, uiTid, ttState, switchId, switchState);
      break;
    }
    case TrainTrackMsgType::SetTrackCmd: {
      if (msg.setTrackCmd.trackId != 0 && msg.setTrackCmd.trackId != 1) {
        logError("invalid track id");
      }
      resetSystem(dispatcherTid, uiTid, ttState);
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
      if (triggeredNode.lock.hasOwner()) {
        marklin::TrainId ownerId = triggeredNode.lock.owner();
        if (auto& st = ttState.getTrain(ownerId).stateMachine; st.type == marklin::TrainStateMachine::Type::Locating) {
          st.locating.hasHitFirstSensor = true;
        }
        marklin::calibrateTrainAndSetSwitches(
            ttState, triggeredNode, currentTicks,
            [&](const marklin::TrackNode& node) { setSwitchNodeByLockState(dispatcherTid, uiTid, ttState, node); });
      } else {
        // TODO: If the sensor has no owner, attribute it to a train currently in Locating state?
      }
      break;
    }
    case TrainTrackMsgType::GotoCmd: {
      marklin::TrainId trainId = msg.gotoCmd.trainId;
      marklin::SpeedLevel speedLevel = msg.gotoCmd.speedLevel;
      marklin::TrackNode* dest = ttState.getTrackNodeByName(msg.gotoCmd.location);
      if (!marklin::isValidTrainId(trainId)) {
        logError("Invalid train id for goto.");
      }
      if (!marklin::isValidSpeedLevel(speedLevel)) {
        logError("Invalid speed level for goto.");
      }
      if (!dest) {
        logError("Invalid track node name for goto.");
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
      train.stateMachine.locating.offset = msg.gotoCmd.offsetMm * 1000;
      train.stateMachine.locating.hasHitFirstSensor = false;
      train.path.clear();

      // Guide the train to enter the loop.
      static constexpr marklin::SwitchId toCurved[] = {3, 5, 8, 9, 11, 12, 14, 15, 18, 154, 155};
      static constexpr marklin::SwitchId toStraight[] = {1, 2, 4, 6, 7, 10, 13, 16, 17, 153, 156};
      for (marklin::SwitchId id : toCurved) {
        broadcastSwitchState(dispatcherTid, uiTid, ttState, id, marklin::SwitchState::Curved);
      }
      for (marklin::SwitchId id : toStraight) {
        broadcastSwitchState(dispatcherTid, uiTid, ttState, id, marklin::SwitchState::Straight);
      }
      broadcastTrainSpeedLevel(dispatcherTid, uiTid, ttState, trainId, speedLevel);
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
          if (!train.lastVisitedNode || !train.stateMachine.locating.hasHitFirstSensor) {
            break;
          }

          // Find the path to the destination.
          marklin::unlockAllLoopSensorNodes(ttState, trainId);
          marklin::Distance totalPathDist = 0;
          if (!marklin::planPath(ttState, trainId, train.stateMachine.locating.dest, totalPathDist)) {
            train.stateMachine.type = marklin::TrainStateMachine::Type::Idle;
            notifyStatusToUI(uiTid, "Train %u failed to find any path.", trainId);
            break;
          }

          // Update the switches directions.
          for (marklin::TrackNode* curr : train.path) {
            setSwitchNodeByLockState(dispatcherTid, uiTid, ttState, *curr);
          }

          train.stateMachine.type = marklin::TrainStateMachine::Type::Pathing;
          train.stateMachine.pathing.dest = train.stateMachine.locating.dest;
          train.stateMachine.pathing.offset = train.stateMachine.locating.offset;
          train.stateMachine.pathing.pathDistance = totalPathDist + train.stateMachine.locating.offset;
          notifyStatusToUI(uiTid, "Train %u found path[%u]: %u um.", trainId, train.path.size(),
                           train.stateMachine.pathing.pathDistance);
          break;
        }
        case marklin::TrainStateMachine::Type::Pathing: {
          // Update train position.
          train.estimatedOffsetFromLast += train.estimatedSpeed;

          // Update estimated node, offset, and remaining distance.
          marklin::Distance estOffsetFromEstNode = train.estimatedOffsetFromLast;
          marklin::TrackNode* last = train.lastVisitedNode;
          for (marklin::TrackNode* curr : train.path) {
            marklin::Distance dist = getAdjacentDistance(*last, *curr);
            if (estOffsetFromEstNode < dist) {
              break;
            }
            estOffsetFromEstNode -= dist;
            train.estimatedNode = curr;
            last = curr;
          }
          train.estimatedOffsetFromEstimatedNode = estOffsetFromEstNode;

          // Stop the train if close to safe stopping distance.
          train.estimatedPathDistance = train.stateMachine.pathing.pathDistance - train.estimatedOffsetFromLast;
          if (train.estimatedPathDistance <= marklin::getStoppingDistance(train.speedLevel)) {
            broadcastTrainSpeedLevel(dispatcherTid, uiTid, ttState, trainId, 0);
            train.stateMachine.type = marklin::TrainStateMachine::Type::Idle;
            notifyStatusToUI(uiTid, "Train %u arrived at destination (estimate).", trainId);
          }
        }
        }

        // Log to UI.
        if (currentTicks - lastTrainUIRefreshTicks >= 10 && !trainStates.full()) {
          trainStates.pushBack(TrainStatesEntry{
              .trainId = trainId,
              .estimatedSpeed = train.estimatedSpeed,
              .lastVisitedNode = train.lastVisitedNode,
              .estimatedOffsetFromLast = train.estimatedOffsetFromLast, // um away from the last visited node.
              .estimatedNode = train.estimatedNode,
              .estimatedOffsetFromEstimatedNode =
                  train.estimatedOffsetFromEstimatedNode,           // um away from the estimated node.
              .estimatedPathDistance = train.estimatedPathDistance, // um away form the destination.
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
