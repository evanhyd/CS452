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
  if (train.navigation.state != marklin::NavigationSystem::State::Idling) {
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
  if (train.navigation.state != marklin::NavigationSystem::State::Idling) {
    notifyStatusToUI(context.uiTid, "Train %u is busy right now.", trainId);
    return;
  }

  // Perform reversing.
  train.navigation.state = marklin::NavigationSystem::State::Reversing;
  train.navigation.reversingTask.preReversingSpeedLevel = train.kinematics.offlineSpeedLevel;
  broadcastTrainSpeedLevel(context, trainId, 0);
  notifyStatusToUI(context.uiTid, "Reversing train %u.", trainId);
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
    marklin::TrainId trainId = context.pfSystem.getReserver(triggeredNode.id);
    if (trainId != marklin::NO_TRAIN) {
      return trainId;
    }

    // Priority 2: Prediction Matching.
    for (marklin::TrainId id : context.activeTrains) {
      marklin::Train& train = context.ttState.getTrain(id);
      if (train.prediction.sensor && train.prediction.sensor->id == triggeredNode.id) {
        return id;
      }
    }

    // Priority 3: Pair with the train in Lost state.
    for (marklin::TrainId id : context.activeTrains) {
      marklin::Train& train = context.ttState.getTrain(id);
      if (train.kinematics.state == marklin::KinematicsSystem::State::Lost) {
        return id;
      }
    }
    return marklin::NO_TRAIN;
  }();
  if (ownerId == marklin::NO_TRAIN) {
    return;
  }
  marklin::Train& train = context.ttState.getTrain(ownerId);

  marklin::Distance dS = [&] -> marklin::Distance {
    if (train.kinematics.state == marklin::KinematicsSystem::State::Tracked) {
      KIT_ASSERT(train.kinematics.lastSensor, "tracked train last sensor is null");
      return marklin::getDistanceBetweenSensor(context.ttState.getCurrentTrackId(), *train.kinematics.lastSensor,
                                               triggeredNode);
    }
    return 0;
  }();

  // Update kinematics values.
  train.kinematics.triggerSensor(triggeredNode, dS, context.currentTicks);

  // Update the sensor prediction.
  marklin::Distance distToNext = 0;
  marklin::TrackNode* nextSensor = marklin::getNextSensor(context.ttState, triggeredNode, distToNext);
  train.prediction.triggerSensor(triggeredNode, nextSensor, distToNext, train.kinematics.estimatedSpeed,
                                 context.currentTicks);
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
  if (train.navigation.state != marklin::NavigationSystem::State::Idling) {
    notifyStatusToUI(context.uiTid, "Train %u is busy right now.", id);
    return;
  }

  train.navigation.findingPathTask = {dest->id, offset, speedLevel};
  train.navigation.state = marklin::NavigationSystem::State::FindingPath;
  broadcastTrainSpeedLevel(context, id, 0);
  notifyStatusToUI(context.uiTid, "Route train %u to %s.", id, dest->name);
} // namespace k4

inline void timerTickHandler(TrainTrackServerContext& context, uint32_t ticks) {
  context.currentTicks = ticks;
  bool shouldUpdateTrainUI = context.currentTicks - context.lastTrainUIRefreshTicks >= 10;

  for (marklin::TrainId trainId : context.activeTrains) {
    marklin::Train& train = context.ttState.getTrain(trainId);

    // Part A: Navigation Block.
    switch (train.navigation.state) {
    case marklin::NavigationSystem::State::Manual: {
      break;
    }
    case marklin::NavigationSystem::State::FindingPath: {
      break;
    }
    case marklin::NavigationSystem::State::Routed: {
      break;
    }
    case marklin::NavigationSystem::State::Reversing: {
      if (train.kinematics.estimatedSpeed == 0) {
      }
      break;
    }

    case marklin::NavigationState::Reversing: {
      if (train.kin.estimatedSpeed == 0 && train.hw.offlineSpeed == 0) {
        train.hw.forward = !train.hw.forward;
        auto reverseDir = train.hw.forward ? marklin::TrainDirection::Forward : marklin::TrainDirection::Backward;
        sendToDispatcher(context.dispatcherTid, marklin::MMessage::setTrainDirection(trainId, reverseDir));
        if (train.kin.lastKnownNode) {
          marklin::Distance remainingOffset = train.kin.estimatedOffsetFromLast;
          marklin::TrackNode* curr = train.kin.lastKnownNode;
          marklin::TrackNode* next = nullptr;
          marklin::Distance edgeDist = 0;
          while ((next = marklin::getNextTrackNode(context.ttState, *curr, edgeDist)) && remainingOffset > edgeDist) {
            remainingOffset -= edgeDist;
            curr = next;
          }
          marklin::TrackNode* revNode = next ? next->reverse : curr->reverse;
          marklin::Distance revOffset = next ? edgeDist - remainingOffset : 0;
          constexpr marklin::Distance TRAIN_LENGTH_UM = 220000;
          revOffset += TRAIN_LENGTH_UM;
          marklin::Distance revEdgeDist = 0;
          marklin::TrackNode* revNext = nullptr;
          while ((revNext = marklin::getNextTrackNode(context.ttState, *revNode, revEdgeDist)) &&
                 revOffset > revEdgeDist) {
            revOffset -= revEdgeDist;
            revNode = revNext;
          }
          train.kin.lastKnownNode = revNode;
          train.kin.estimatedOffsetFromLast = revOffset;
        }
        train.navigationState = train.nav.resumeNavigationState;
        if (train.navigationState == marklin::NavigationState::Manual) {
          broadcastTrainSpeedLevel(context, trainId, train.nav.resumeSpeed);
        }
      }
      break;
    }
    case marklin::NavigationState::FindingPath: {
      if (train.kinematicState == marklin::KinematicState::Lost) {
        broadcastTrainSpeedLevel(context, trainId, train.nav.resumeSpeed);
        break;
      }

      // Unlock all owned nodes.
      for (auto& ownedNodes = context.pfSystem.getOwned(trainId); !ownedNodes.empty();
           context.pfSystem.unlock(ownedNodes.front(), trainId))
        ;
      train.nav.nodeAheadLocked = 0;
      train.nav.path.clear();
      train.navigationState = marklin::NavigationState::Routed;

      bool foundForward = context.pfSystem.planPath(context.ttState, trainId, train.nav.dest, train.nav.pathDistance);
      bool foundReverse = false;
      if (!foundForward) {
        train.nav.path.clear();
        train.kin.lastKnownNode = train.kin.lastKnownNode->reverse;
        foundReverse = context.pfSystem.planPath(context.ttState, trainId, train.nav.dest, train.nav.pathDistance);
        train.kin.lastKnownNode = train.kin.lastKnownNode->reverse;
      }
      if (foundForward) {
        train.navigationState = marklin::NavigationState::Routed;
        broadcastTrainSpeedLevel(context, trainId, train.nav.resumeSpeed);
        notifyStatusToUI(context.uiTid, "Train %u routed forward, path distance: %d", trainId, train.nav.pathDistance);
      } else if (foundReverse) {
        train.nav.path.clear();
        train.navigationState = marklin::NavigationState::Reversing;
        train.nav.resumeNavigationState = marklin::NavigationState::FindingPath;
        broadcastTrainSpeedLevel(context, trainId, 0);
        notifyStatusToUI(context.uiTid, "Train %u must reverse to start path.", trainId);
      } else {
        train.navigationState = marklin::NavigationState::Manual;
        broadcastTrainSpeedLevel(context, trainId, 0);
        notifyStatusToUI(context.uiTid, "Train %u no path found.", trainId);
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
      static constexpr marklin::Speed ACCEL_UM_PER_TICK_PER_TICK = 17;
      marklin::Speed speedDiff = targetSpeed - train.hw.offlineSpeed;
      marklin::Speed delta = kit::clamp(speedDiff, -ACCEL_UM_PER_TICK_PER_TICK, ACCEL_UM_PER_TICK_PER_TICK);
      if (train.hw.offlineSpeed == 0) {
        train.kin.estimatedSpeed += delta;
      } else {
        train.kin.estimatedSpeed = (train.kin.estimatedSpeed * (train.hw.offlineSpeed + delta)) / train.hw.offlineSpeed;
      }
      train.hw.offlineSpeed += delta;
      train.kin.estimatedSpeed = kit::max(0, train.kin.estimatedSpeed);
    } else if (train.hw.offlineSpeed == 0) {
      train.kin.estimatedSpeed = 0;
    }

    // Apply the estimated speed to update the estimated position.
    if (train.kinematicState == marklin::KinematicState::Tracked) {
      train.kin.estimatedOffsetFromLast += train.kin.estimatedSpeed;
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
          notifyStatusToUI(context.uiTid, "Train %u halted at %s.", trainId, prev->name);
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
