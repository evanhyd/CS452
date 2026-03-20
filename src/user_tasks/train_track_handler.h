#pragma once
#include "marklin/marklin_pathfinding.h"
#include "marklin/marklin_train_track.h"
#include "train_track_server_context.h"
#include "train_track_util.h"
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
      if (train.kinematicState == marklin::KinematicState::Lost && train.hw.speedLevel > 0) {
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
    for (marklin::TrackNode* tail = train.kinematics.lastKnownNode; tail->id != triggeredNode.id;) {
      KIT_ASSERT(context.pfSystem.getOwner(tail->id) == ownerId, "train has no path ownership");
      context.pfSystem.unlock(tail->id, ownerId);
      context.pfSystem.unreserve(tail->id, ownerId);
      --train.nav.nodeAheadLocked;
      --train.nav.nodeAheadReserved;

      marklin::Distance dist = 0;
      tail = marklin::getNextTrackNode(context.ttState, *tail, dist);
      dS += dist;
      KIT_ASSERT(tail, "tail must lead to triggered node");
    }
  }

  // Clean up routed path.
  if (train.navigationState == marklin::NavigationState::Routing) {
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
    KIT_ASSERT(dS > 0, "tail to triggered node path length is 0");
    uint32_t dT = kit::max(1u, context.currentTicks - train.kinematics.lastKnownTicks);
    train.kinematics.updateSpeed(dS, dT);
  }
  train.kinematics.lastKnownTicks = context.currentTicks;
  train.kinematics.lastKnownNode = &triggeredNode;
  train.kinematics.estimatedOffsetFromLast = 0;
  train.kinematics.estimatedNode = &triggeredNode;
  train.kinematics.estimatedOffsetFromEstimatedNode = 0;

  // Update the sensor prediction.
  if (train.prediction.predictedNextSensor && train.prediction.predictedNextSensor->id == triggeredNode.id) {
    train.prediction.lastTimeErrorTicks =
        static_cast<int32_t>(context.currentTicks) - static_cast<int32_t>(train.prediction.predictedNextSensorTicks);
    train.prediction.lastDistErrorUm = train.prediction.lastTimeErrorTicks * train.kinematics.estimatedSpeed;
  } else {
    train.prediction.lastTimeErrorTicks = 0;
    train.prediction.lastDistErrorUm = 0;
  }

  marklin::Distance distToNext = 0;
  train.prediction.predictedNextSensor = marklin::getNextSensor(context.ttState, triggeredNode, distToNext);
  if (train.prediction.predictedNextSensor && train.kinematics.estimatedSpeed > 0) {
    train.prediction.predictedNextSensorTicks =
        context.currentTicks + static_cast<uint32_t>(distToNext / train.kinematics.estimatedSpeed);
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

  // TODO: fix it later
  //   train.nav.navDest = dest->id;
  //   train.nav.navOffset = offset;

  //   if (train.kinematicState == marklin::KinematicState::Tracked) {
  //     // Already tracked — skip locating, just start routing.
  //     train.navigationState = marklin::NavigationState::Routing;
  //     broadcastTrainSpeedLevel(context, id, speedLevel);
  //     notifyStatusToUI(context.uiTid, "Routing tracked train %u.", id);
  //   } else {
  //     // Lost — need to locate first.
  //     if (!marklin::lockAllLoopSensorNodes(context.ttState, id)) {
  //       notifyStatusToUI(context.uiTid, "Train %u can not acquire an occupied loop.", id);
  //       return;
  //     }
  //     train.kinematicState = marklin::KinematicState::Locating;
  //     train.navigationState = marklin::NavigationState::Routing;

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
    case marklin::NavigationState::Manual: {
      // Manual does nothing.
      break;
    }
    case marklin::NavigationState::Yielding: {
      // Track clearance and recovery is handled by Part C.
      break;
    }
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
    case marklin::NavigationState::Routing: {
      // TODO: add it back later
      //   marklin::assertTrainState(train);

      //   if (train.kinematicState == marklin::KinematicState::Locating) {
      //     marklin::unlockAllLoopSensorNodes(context.ttState, trainId);
      //     train.kinematicState = marklin::KinematicState::Tracked;
      //   }

      //   // Find the path to the destination.
      //   marklin::Distance totalPathDist = 0;
      //   if (!marklin::planPath(context.ttState, trainId, train.nav.navDest, totalPathDist)) {
      //     broadcastTrainSpeedLevel(context, trainId, 0);
      //     train.navigationState = marklin::NavigationState::Manual;
      //     notifyStatusToUI(context.uiTid, "Train %u failed to find any path.", trainId);
      //     break;
      //   }

      //   // Update the switches directions.
      //   for (marklin::TrackNode* curr : train.nav.path) {
      //     setSwitchNodeByLockState(context, *curr);
      //   }

      //   train.navigationState = marklin::NavigationState::Halting;
      //   train.nav.navPathDistance = totalPathDist + train.nav.navOffset;
      //   notifyStatusToUI(context.uiTid, "Train %u found path[%u]: %u um.", trainId, train.nav.path.size(),
      //                    train.nav.navPathDistance);
      break;
    }
    case marklin::NavigationState::Halting: {
      // TODO: add it back later.
      //   // Stop the train if close to safe stopping distance.
      //   train.nav.estimatedPathDistance = train.nav.navPathDistance - train.kinematics.estimatedOffsetFromLast;
      //   if (train.nav.estimatedPathDistance <= marklin::getStoppingDistance(train.hw.speedLevel)) {
      //     notifyStatusToUI(context.uiTid, "Train %u arrived at destination %s[%u um].", trainId,
      //                      train.nav.path.back()->name, train.nav.navOffset);

      //     broadcastTrainSpeedLevel(context, trainId, 0);

      //     // Unlock the remaining path.
      //     train.kinematics.lastKnownNode->lock.release(trainId);
      //     for (marklin::TrackNode* node : train.nav.path) {
      //       node->lock.release(trainId);
      //     }

      //     // Clear navigation meta data only; keep kinematic state intact.
      //     train.nav.estimatedPathDistance = 0;
      //     train.navigationState = marklin::NavigationState::Manual;
      //     train.nav.path.clear();
      //   }
      break;
    }
    }

    // Part B: Kinematics Block.
    // Apply acceleration to update the estimated speed until the offlineSpeed is close to the targetSpeed.
    if (marklin::Speed targetSpeed = marklin::convertSpeedLevelToOfflineSpeed(train.hw.speedLevel);
        train.hw.offlineSpeed != targetSpeed) {
      static constexpr marklin::Speed ACCEL_UM_PER_TICK_PER_TICK = 19;
      marklin::Speed speedDiff = targetSpeed - train.hw.offlineSpeed;
      marklin::Speed delta = speedDiff < 0 ? kit::min(ACCEL_UM_PER_TICK_PER_TICK, speedDiff)
                                           : kit::max(-ACCEL_UM_PER_TICK_PER_TICK, speedDiff);
      train.hw.offlineSpeed += delta;
      train.kinematics.estimatedSpeed += delta;
    } else if (train.hw.offlineSpeed == 0) {
      train.kinematics.estimatedSpeed = 0;
    }

    // Apply the estimated speed to update the estimated position.
    if (train.kinematicState == marklin::KinematicState::Tracked) {
      train.kinematics.estimatedOffsetFromLast += train.kinematics.estimatedSpeed;
      marklin::Distance offset = train.kinematics.estimatedOffsetFromLast;
      marklin::TrackNode* last = train.kinematics.lastKnownNode;
      KIT_ASSERT(last, "tracked train last known node must be non null");
      for (;;) {
        marklin::Distance dist = 0;
        marklin::TrackNode* next = marklin::getNextTrackNode(context.ttState, *last, dist);
        if (!next || offset < dist) {
          train.kinematics.estimatedNode = last;
          train.kinematics.estimatedOffsetFromEstimatedNode = offset;
          break;
        }
        offset -= dist;
        last = next;
      }
    }

    // Part C: Dynamic Lookahead Reservation.
    if (train.kinematicState == marklin::KinematicState::Tracked && train.hw.speedLevel > 0) {
      marklin::Distance lookaheadRemaining =
          marklin::getStoppingDistance(train.navigationState == marklin::NavigationState::Yielding
                                           ? train.nav.resumeSpeed
                                           : train.hw.speedLevel) +
          100'000; // 10 cm

      marklin::TrackNode* prev = train.kinematics.lastKnownNode;
      auto pathIt = train.nav.path.begin();
      bool isRouting = (train.navigationState == marklin::NavigationState::Routing);
      bool pathClear = true;

      for (int depth = 0; lookaheadRemaining > 0; ++depth) {
        marklin::Distance edgeDist = 0;
        marklin::TrackNode* curr = nullptr;

        if (pathIt != train.nav.path.end()) {
          // Routing mode uses the existing path.
          curr = *pathIt++;
          edgeDist = marklin::getAdjacentDistance(*prev, *curr);
          if (prev->type == marklin::TrackNode::Type::Branch) {
            marklin::SwitchState desiredState =
                (marklin::getAdjacentDirection(*prev, *curr) == marklin::TrackDirection::Curved)
                    ? marklin::SwitchState::Curved
                    : marklin::SwitchState::Straight;
            broadcastSwitchState(context, prev->num, desiredState);
          }
        } else {
          // Run out of path in routing mode. Must halt at the destination.
          if (isRouting) {
            train.navigationState = marklin::NavigationState::Halting;
            isRouting = false;
            notifyStatusToUI(context.uiTid, "Train %u halt because close to destination.", trainId);
          }

          // Use the next node if in manual mode or run out of path in routing.
          curr = marklin::getNextTrackNode(context.ttState, *prev, edgeDist);
        }

        // Reach the exit. Must halt.
        if (!curr) {
          pathClear = false;
          if (train.navigationState != marklin::NavigationState::Halting) {
            train.navigationState = marklin::NavigationState::Halting;
            broadcastTrainSpeedLevel(context, trainId, 0);
            notifyStatusToUI(context.uiTid, "Train %u hatl because close to end of track.", trainId);
          }
          break;
        }

        lookaheadRemaining -= (depth == 0) ? (edgeDist - train.kinematics.estimatedOffsetFromLast) : edgeDist;

        // Reserve and lock in manual mode. Switch to yield if fails.
        bool needsReserve = (!isRouting && depth >= train.nav.nodeAheadReserved);
        bool needsLock = (depth >= train.nav.nodeAheadLocked);
        if ((needsReserve && !context.pfSystem.canReserve(*curr, trainId)) ||
            (needsLock && !context.pfSystem.canLock(curr->id, trainId))) {
          pathClear = false;
          if (train.navigationState != marklin::NavigationState::Yielding) {
            train.nav.resumeSpeed = train.hw.speedLevel;
            train.nav.resumeNavigationState = train.navigationState;
            train.navigationState = marklin::NavigationState::Yielding;
            broadcastTrainSpeedLevel(context, trainId, 0);
            notifyStatusToUI(context.uiTid, "Train %u yielding at %s.", trainId, curr->name);
          }
          break;
        }

        // Reserve and lock.
        if (needsReserve) {
          context.pfSystem.reserve(curr->id, trainId);
          train.nav.nodeAheadReserved++;
        }
        if (needsLock) {
          context.pfSystem.lock(curr->id, trainId);
          train.nav.nodeAheadLocked++;
        }
        prev = curr;
      }

      // If the train was in yield but the path is clear now, then restore the old state.
      if (train.navigationState == marklin::NavigationState::Yielding && pathClear) {
        train.navigationState = train.nav.resumeNavigationState;
        broadcastTrainSpeedLevel(context, trainId, train.nav.resumeSpeed);
        notifyStatusToUI(context.uiTid, "Track cleared. Train %u resuming.", trainId);
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
