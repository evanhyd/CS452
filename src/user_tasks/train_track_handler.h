#pragma once
#include "marklin/marklin_pathfinding.h"
#include "marklin/marklin_train_track.h"
#include "train_track_server_context.h"
#include "train_track_util.h"
#include "user_tasks/send_util.h"
#include "util/debug.h"

namespace k4 {

inline void setSpeedCmdHandler(TrainTrackServerContext& context, marklin::TrainId trainId,
                               marklin::SpeedLevel speedLevel) {
  KIT_ASSERT(marklin::isValidTrainId(trainId), "invalid train id");
  KIT_ASSERT(marklin::isValidSpeedLevel(speedLevel), "invalid train speed level");

  // Check busy status.
  initTrain(context, trainId);
  marklin::Train& train = context.ttState.getTrain(trainId);
  if (train.navigationState != marklin::NavigationState::Manual) {
    notifyStatusToUI(context.uiTid, "Train %u is busy right now.", trainId);
    return;
  }

  // Change speed.
  broadcastTrainSpeedLevel(context, trainId, speedLevel);
  notifyStatusToUI(context.uiTid, "Set train %u to speed %u.", trainId, speedLevel);
}

inline void reverseCmdHandler(TrainTrackServerContext& context, marklin::TrainId trainId) {
  KIT_ASSERT(marklin::isValidTrainId(trainId), "invalid train id");

  // Check busy status.
  initTrain(context, trainId);
  marklin::Train& train = context.ttState.getTrain(trainId);
  if (train.navigationState != marklin::NavigationState::Manual) {
    notifyStatusToUI(context.uiTid, "Train %u is busy right now.", trainId);
    return;
  }

  // Perform reversing.
  train.navigationState = marklin::NavigationState::Reversing;
  train.nav.reverseCountdownTicks = 50 + static_cast<unsigned>(train.hw.speedLevel) * 25;
  sendToDispatcher(context.dispatcherTid, marklin::MMessage::setTrainSpeed(trainId, 0));
  notifyStatusToUI(context.uiTid, "Reversing train %u in %u ticks.", trainId, train.nav.reverseCountdownTicks);
}

inline void setSwitchCmdHandler(TrainTrackServerContext& context, marklin::SwitchId switchId,
                                marklin::SwitchState state) {
  KIT_ASSERT(marklin::isValidSwitchId(switchId), "invalid switch id");
  broadcastSwitchState(context, switchId, state);
}

inline void setTrackCmdHandler(TrainTrackServerContext& context, marklin::TrackId trackId) {
  KIT_ASSERT(marklin::isValidTrack(trackId), "invalid track id");
  resetContext(context);
  context.ttState.setCurrentTrack(trackId);
  notifyStatusToUI(context.uiTid, "Switched track to %c.", (trackId == 0 ? 'A' : 'B'));
}

inline void sensorEventHandler(TrainTrackServerContext& context, const marklin::SensorTriggeredEvent& sensorEvent) {
  if (sensorEvent.state != marklin::SensorState::Occupied) {
    return;
  }

  // Send sensor to UI.
  SensorHistoryEntry newEntry{};
  newEntry.event = sensorEvent;
  newEntry.ticks = context.currentTicks;
  context.sensorHistory.push(newEntry);
  sendSensorHistoryToUI(context);

  // Obtain the train that triggers the sensor.
  marklin::TrackNode& triggeredNode = context.ttState.getTrackNodeById(sensorEvent.id);
  marklin::TrainId ownerId = [&] -> marklin::TrainId {
    // Priority 1: Explicit Lock Ownership.
    marklin::TrainId trainId = context.pfSystem.getOwner(triggeredNode.id);
    if (trainId != marklin::NO_TRAIN) {
      return trainId;
    }

    // Priority 2: Prediction Matching.
    for (marklin::TrainId id : context.activeTrains) {
      marklin::Train& train = context.ttState.getTrain(id);
      if (train.prediction.predictedNextSensor && train.prediction.predictedNextSensor->id == triggeredNode.id) {
        return id;
      }
    }

    // Priority 3: Pair with the train in Lost state.
    for (marklin::TrainId id : context.activeTrains) {
      marklin::Train& train = context.ttState.getTrain(id);
      if (train.kinematicState == marklin::KinematicState::Lost) {
        return id;
      }
    }
    return marklin::NO_TRAIN;
  }();
  if (ownerId == marklin::NO_TRAIN) {
    return;
  }

  marklin::Train& train = context.ttState.getTrain(ownerId);
  marklin::assertTrainState(train);

  // Tail clearance and calculate the tail path length.
  marklin::Distance dS = 0;
  if (train.kinematicState == marklin::KinematicState::Tracked) {
    KIT_ASSERT(triggeredNode.id != train.kin.lastKnownNode->id, "hit the same sensor twice in a row");

    auto& ownedQueue = context.pfSystem.getOwned(ownerId);
    for (marklin::TrackNode* tail = train.kin.lastKnownNode; tail->id != triggeredNode.id;) {
      KIT_ASSERT(!ownedQueue.empty(), "Train triggered a node it never locked");
      marklin::TrackNodeId lockedNodeId = ownedQueue.front();
      marklin::TrackNode& lockedNode = context.ttState.getTrackNodeById(lockedNodeId);
      dS += marklin::getAdjacentDistance(*tail, lockedNode);
      tail = &lockedNode;
      context.pfSystem.unlock(lockedNodeId, ownerId);
      --train.nav.nodeAheadLocked;
    }
  }

  // Clean up routed path.
  if (train.navigationState == marklin::NavigationState::Routed ||
      train.nav.resumeNavigationState == marklin::NavigationState::Routed) {
    while (!train.nav.path.empty()) {
      marklin::TrackNode* curr = train.nav.path.popFront();
      if (curr->id == triggeredNode.id) {
        break;
      }
    }
    train.nav.pathDistance -= dS;
    train.nav.estimatedPathDistance = train.nav.pathDistance;
  }

  // Update kinematics values.
  if (train.kinematicState == marklin::KinematicState::Tracked) {
    uint32_t dT = kit::max(1u, context.currentTicks - train.kin.lastKnownTicks);
    train.kin.updateSpeed(dS, dT);
  }
  train.kin.lastKnownTicks = context.currentTicks;
  train.kin.lastKnownNode = &triggeredNode;
  train.kin.estimatedOffsetFromLast = 0;

  // Update the sensor prediction.
  if (train.prediction.predictedNextSensor && train.prediction.predictedNextSensor->id == triggeredNode.id) {
    train.prediction.lastTimeErrorTicks =
        static_cast<int32_t>(context.currentTicks) - static_cast<int32_t>(train.prediction.predictedNextSensorTicks);
    train.prediction.lastDistErrorUm = train.prediction.lastTimeErrorTicks * train.kin.estimatedSpeed;
  } else {
    train.prediction.lastTimeErrorTicks = 0;
    train.prediction.lastDistErrorUm = 0;
  }

  marklin::Distance distToNext = 0;
  train.prediction.predictedNextSensor = marklin::getNextSensor(context.ttState, triggeredNode, distToNext);
  if (train.prediction.predictedNextSensor && train.kin.estimatedSpeed > 0) {
    train.prediction.predictedNextSensorTicks =
        context.currentTicks + static_cast<uint32_t>(distToNext / train.kin.estimatedSpeed);
  } else {
    train.prediction.predictedNextSensorTicks = 0;
  }

  // Update kinematics state.
  if (train.kinematicState == marklin::KinematicState::Lost) {
    train.kinematicState = marklin::KinematicState::Tracked;
    notifyStatusToUI(context.uiTid, "Train %u tracked at %s.", ownerId, triggeredNode.name);
  }
  marklin::assertTrainState(train);
}

inline void gotoCmdHandler(TrainTrackServerContext& context, marklin::TrainId id, marklin::SpeedLevel speedLevel,
                           const char* location, marklin::Distance offset) {
  KIT_ASSERT(marklin::isValidTrainId(id), "Invalid train id for goto.");
  KIT_ASSERT(marklin::isValidSpeedLevel(speedLevel), "Invalid speed level for goto.");
  marklin::TrackNode* dest = context.ttState.getTrackNodeByName(location);
  KIT_ASSERT(dest, "Invalid track node name for goto.");

  // Check busy status.
  initTrain(context, id);
  marklin::Train& train = context.ttState.getTrain(id);
  if (train.navigationState != marklin::NavigationState::Manual) {
    notifyStatusToUI(context.uiTid, "Train %u is busy right now.", id);
    return;
  }

  train.nav.dest = dest->id;
  train.nav.offset = offset;
  train.navigationState = marklin::NavigationState::FindingPath;
  broadcastTrainSpeedLevel(context, id, speedLevel);
  notifyStatusToUI(context.uiTid, "Route train %u to %s.", id, dest->name);
  //   {
  //     // Guide the train to enter the loop.
  //     static constexpr marklin::SwitchId toCurved[] = {3, 5, 8, 9, 11, 12, 14, 15, 18, 154, 155};
  //     static constexpr marklin::SwitchId toStraight[] = {1, 2, 4, 6, 7, 10, 13, 16, 17, 153, 156};
  //     for (marklin::SwitchId switchId : toCurved) {
  //       broadcastSwitchState(context, switchId, marklin::SwitchState::Curved);
  //     }
  //     for (marklin::SwitchId switchId : toStraight) {
  //       broadcastSwitchState(context, switchId, marklin::SwitchState::Straight);
  //     }
  //     broadcastTrainSpeedLevel(context, id, speedLevel);
  //     notifyStatusToUI(context.uiTid, "Locating train %u.", id);
  //   }
}

inline void timerTickHandler(TrainTrackServerContext& context, uint32_t ticks) {
  context.currentTicks = ticks;
  bool shouldUpdateTrainUI = context.currentTicks - context.lastTrainUIRefreshTicks >= 10;

  for (marklin::TrainId trainId : context.activeTrains) {
    marklin::Train& train = context.ttState.getTrain(trainId);

    // Part A: Navigation Block.
    switch (train.navigationState) {
    case marklin::NavigationState::Reversing: {
      --train.nav.reverseCountdownTicks;
      if (train.nav.reverseCountdownTicks == 0) {
        train.navigationState = marklin::NavigationState::Manual;
        train.hw.forward = !train.hw.forward;
        auto reverseDir = train.hw.forward ? marklin::TrainDirection::Forward : marklin::TrainDirection::Backward;
        sendToDispatcher(context.dispatcherTid, marklin::MMessage::setTrainDirection(trainId, reverseDir));
        sendToDispatcher(
            context.dispatcherTid,
            marklin::MMessage::setTrainSpeed(trainId, marklin::convertSpeedLevelToCANSpeed(train.hw.speedLevel)));
      }
      break;
    }
    case marklin::NavigationState::FindingPath: {
      if (train.kinematicState == marklin::KinematicState::Lost) {
        break;
      }

      // Unlock all owned nodes.
      for (auto& ownedNodes = context.pfSystem.getOwned(trainId); !ownedNodes.empty();
           context.pfSystem.unlock(ownedNodes.front(), trainId))
        ;
      train.nav.nodeAheadLocked = 0;
      train.nav.path.clear();
      train.navigationState = marklin::NavigationState::Routed;
      if (!context.pfSystem.planPath(context.ttState, trainId, train.nav.dest, train.nav.pathDistance)) {
        logError("Train failed to find a path.");
      }
      break;
    }
    default:
      break;
    }

    // Part B: Kinematics Block.
    // Apply acceleration to update the estimated speed until the offlineSpeed is close to the targetSpeed.
    if (marklin::Speed targetSpeed = marklin::convertSpeedLevelToOfflineSpeed(train.hw.speedLevel);
        train.hw.offlineSpeed != targetSpeed) {
      static constexpr marklin::Speed ACCEL_UM_PER_TICK_PER_TICK = 19;
      marklin::Speed speedDiff = targetSpeed - train.hw.offlineSpeed;
      marklin::Speed delta = kit::clamp(speedDiff, -ACCEL_UM_PER_TICK_PER_TICK, ACCEL_UM_PER_TICK_PER_TICK);
      train.hw.offlineSpeed += delta;
      train.kin.estimatedSpeed += delta;
      train.kin.estimatedSpeed = kit::max(0, train.kin.estimatedSpeed);
    } else if (train.hw.offlineSpeed == 0) {
      train.kin.estimatedSpeed = 0;
    }

    // Apply the estimated speed to update the estimated position.
    if (train.kinematicState == marklin::KinematicState::Tracked) {
      train.kin.estimatedOffsetFromLast += train.kin.estimatedSpeed;
      marklin::Distance dist = 0;
      marklin::getNextTrackNode(context.ttState, *train.kin.lastKnownNode, dist);
      train.kin.estimatedOffsetFromLast = kit::max(train.kin.estimatedOffsetFromLast, dist);
    }

    // Part C: Dynamic Lookahead Reservation.
    if (train.kinematicState == marklin::KinematicState::Tracked) {
      marklin::Distance lookaheadRemaining =
          train.kin.estimatedOffsetFromLast + marklin::getStoppingDistance(train.kin.estimatedSpeed);

      marklin::TrackNode* prev = train.kin.lastKnownNode;
      auto pathIt = train.nav.path.begin();
      bool pathClear = true;

      for (int depth = 0; lookaheadRemaining > 0; ++depth) {
        marklin::Distance edgeDist = 0;
        marklin::TrackNode* curr = nullptr;
        if (train.navigationState == marklin::NavigationState::Manual ||
            (train.navigationState == marklin::NavigationState::Yielding &&
             train.nav.resumeNavigationState == marklin::NavigationState::Manual)) {
          // Manual or was in manual.
          curr = marklin::getNextTrackNode(context.ttState, *prev, edgeDist);
        } else {
          // Routing or was in routing.
          if (pathIt != train.nav.path.end()) {
            curr = *pathIt++;
            edgeDist = marklin::getAdjacentDistance(*prev, *curr);
          } else {
            curr = nullptr;
          }
        }

        // Reach the exit or the end of the routed path. Must stop.
        if (!curr) {
          pathClear = false;
          for (auto& ownedNodes = context.pfSystem.getOwned(trainId); !ownedNodes.empty();
               context.pfSystem.unlock(ownedNodes.front(), trainId))
            ;
          train.nav.nodeAheadLocked = 0;
          train.nav.path.clear();
          train.navigationState = marklin::NavigationState::Manual;
          broadcastTrainSpeedLevel(context, trainId, 0);
          notifyStatusToUI(context.uiTid, "Train %u hatled at %s.", trainId, prev->name);
          break;
        }

        lookaheadRemaining -= edgeDist;

        // Reserve and lock in manual mode. Switch to yield if fails.
        bool needsLock = (depth >= train.nav.nodeAheadLocked);

        if (needsLock && !context.pfSystem.canLock(curr->id, trainId)) {
          pathClear = false;
          if (train.navigationState != marklin::NavigationState::Yielding) {
            train.nav.resumeSpeed = train.hw.speedLevel;
            train.nav.resumeNavigationState = train.navigationState;
            train.navigationState = marklin::NavigationState::Yielding;
            broadcastTrainSpeedLevel(context, trainId, 0);
          }
          break;
        }

        // Reserve and lock.
        if (needsLock) {
          // Severe train desync. It needs to hit a sensor and clear the locks before acquiring more.
          if (context.pfSystem.getOwned(trainId).full()) {
            break;
          }
          context.pfSystem.lock(curr->id, trainId);
          train.nav.nodeAheadLocked++;
        }

        // Set switch direction in routed mode.
        if ((train.navigationState == marklin::NavigationState::Routed ||
             train.nav.resumeNavigationState == marklin::NavigationState::Routed) &&
            prev->type == marklin::TrackNode::Type::Branch) {
          marklin::SwitchState desiredState =
              (marklin::getAdjacentDirection(*prev, *curr) == marklin::TrackDirection::Curved)
                  ? marklin::SwitchState::Curved
                  : marklin::SwitchState::Straight;
          broadcastSwitchState(context, prev->num, desiredState);
        }
        prev = curr;
      }

      // If the train was in yield but the path is clear now, then restore the old state.
      if (train.navigationState == marklin::NavigationState::Yielding && pathClear) {
        train.navigationState = train.nav.resumeNavigationState;
        broadcastTrainSpeedLevel(context, trainId, train.nav.resumeSpeed);
      }
    }

    // Log to UI.
    if (shouldUpdateTrainUI) {
      KIT_ASSERT(!context.trainStates.full(), "train state buffer overflow");
      TrainStatesEntry entry{.trainId = trainId, .train = &train, .lockedNodes = {}, .lockedNodeCount = 0};
      for (marklin::TrackNodeId nid : context.pfSystem.getOwned(trainId)) {
        entry.lockedNodes[entry.lockedNodeCount++] = &context.ttState.getTrackNodeById(nid);
      }
      context.trainStates.pushBack(entry);
    }
  }

  // Send the train states to the UI.
  if (shouldUpdateTrainUI) {
    context.lastTrainUIRefreshTicks = context.currentTicks;
    sendTrainHistoryToUI(context);
    context.trainStates.clear();
  }
}

} // namespace k4
